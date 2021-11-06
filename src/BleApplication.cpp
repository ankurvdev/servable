// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2000-2001  Qualcomm Incorporated
 *  Copyright (C) 2002-2003  Maxim Krasnyansky <maxk@qualcomm.com>
 *  Copyright (C) 2002-2010  Marcel Holtmann <marcel@holtmann.org>
 *
 *
 */
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <glib.h>

#include <dbus/dbus.h>

extern "C"
{
#include "lib/bluetooth.h"

#include "gdbus/gdbus.h"
#define class deviceclass

#include "lib/bluetooth.h"
#include "lib/hci.h"
#include "lib/hci_lib.h"
#include "lib/l2cap.h"
#include "lib/uuid.h"

#include "src/shared/att.h"
#include "src/shared/gatt-db.h"
#include "src/shared/gatt-server.h"
#include "src/shared/mainloop.h"
#include "src/shared/queue.h"
#include "src/shared/timeout.h"
#include "src/shared/util.h"

// Extra headers
#include "lib/mgmt.h"
#include "log.h"
#include "shared/ad.h"
#include "shared/mgmt.h"
#include "src/adapter.h"
#include "src/gatt-database.h"
#include "src/hcid.h"

#undef class
}

#define DEFAULT_PAIRABLE_TIMEOUT 0       /* disabled */
#define DEFAULT_DISCOVERABLE_TIMEOUT 180 /* 3 minutes */
#define DEFAULT_TEMPORARY_TIMEOUT 30     /* 30 seconds */

#define SHUTDOWN_GRACE_SECONDS 10
#include "BleApplication.h"

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

static std::mutex globalMutex;
static bool       restart = true;
using time_point          = std::chrono::time_point<std::chrono::system_clock>;
using namespace std::chrono_literals;

template <typename T> T* check_not_null(T* val, std::string_view const& errMessage)
{
    if (val == nullptr) throw std::runtime_error(std::string(errMessage));
    return val;
}

int check_rc(int rc, std::string_view const& errMessage)
{
    if (rc < 0) throw std::runtime_error(std::string(errMessage));
    return rc;
}

namespace std
{
template <> struct hash<blegatt::UUID>
{
    size_t operator()(blegatt::UUID const& obj) const { return std::hash<std::string>{}(obj); }
};
}    // namespace std

struct FreeDeleter
{
    void operator()(void* ptr) const { free(ptr); }
};

template <typename T> struct CMem
{
    template <typename TFunc> CMem(TFunc&& f)
    {
        ptr = reinterpret_cast<T*>(f());
        if (ptr == nullptr) { throw std::runtime_error("Empty Return"); }
    }
    ~CMem() { free(ptr); }

    operator void*() { return reinterpret_cast<void*>(ptr); }

    T* operator->() const noexcept { return ptr; }

    T* ptr{};
};

struct FileDescriptor
{
    FileDescriptor(int fd) : _fd(fd)
    {
        if (_fd < 0) throw std::runtime_error("Failed to accept L2CAP ATT connection");
    }
    ~FileDescriptor() { close(_fd); }
    operator int() const { return _fd; }

    int _fd{};
};

void btd_exit(void)
{
    mainloop_quit();
}

namespace Bluez
{
struct Characteristic;

struct Application : blegatt::IBackendHandler
{
    Application(int index);
    ~Application()
    {
        if (gatt) bt_gatt_server_unref(gatt);
        if (db) gatt_db_unref(db);
        if (mgmt) mgmt_unref(mgmt);
    }

    void Clear();
    void Init(int fd, uint16_t mtu);

    uint16_t     adapterIndex{0};
    bdaddr_t     addr;
    int          addr_type;
    struct mgmt* mgmt{nullptr};

    int                    fd;
    struct bt_att*         att{nullptr};
    struct gatt_db*        db{nullptr};
    struct bt_gatt_server* gatt{nullptr};
    blegatt::IApplication* app{nullptr};
    uint16_t               gatt_svc_chngd_handle{};
    bool                   svc_chngd_enabled{false};
};

struct Service : blegatt::IBackendHandler
{
    Application*       app{nullptr};
    blegatt::IService* svc{nullptr};
    uint16_t           handle{};
    gatt_db_attribute* attr{nullptr};
};

struct Characteristic : blegatt::IBackendHandler
{
    Application*              app{nullptr};
    Service*                  svc{nullptr};
    blegatt::ICharacteristic* chrc{nullptr};
    uint16_t                  handle{};
    uint16_t                  ccchandle{};
    time_point                notificationQueuedAt{};
    time_point                lastNotified{};
    std::atomic<uint32_t>     pendingNotifications{};
    struct gatt_db_attribute* attr{nullptr};
    struct gatt_db_attribute* cccattr{nullptr};

    bool SendNotification()
    {
        auto pendingCount = this->pendingNotifications.exchange(0);
        if (pendingCount > 0) try
            {
                std::unique_lock<std::mutex> lock(globalMutex);
                auto                         now = time_point::clock::now();

                uint8_t buffer[64];
                auto    bufsize = this->chrc->ReadValue(std::span(buffer, std::size(buffer)));
                if (!bt_gatt_server_send_notification(this->app->gatt, this->handle, buffer, bufsize, false))
                {
                    throw std::runtime_error("Cannot send a notification");
                }

                this->lastNotified = now;
                return true;
            } catch (std::exception const& ex)
            {
                std::cerr << "Faied to Send Notification for : " << (std::string)(this->chrc->GetUUID()) << " :: " << ex.what()
                          << std::endl;
            }
        return false;
    }

    void MarkNotificationPending()
    {
        if (pendingNotifications == 0) { notificationQueuedAt = time_point::clock::now(); }

        pendingNotifications++;

        /* TODO */ timeout_add(
            1, [](void* ptr) { return reinterpret_cast<Characteristic*>(ptr)->SendNotification(); }, this, NULL);

        // app->WakeUpNotificationHandler();
    }

    static void ReadCCCCallback(gatt_db_attribute* attrib, uint id, uint16_t offset, uint8_t opcode, bt_att* att, void* cbptr) noexcept
    try
    {
        std::unique_lock<std::mutex> lock(globalMutex);

        uint8_t error   = offset > 2 ? BT_ATT_ERROR_INVALID_OFFSET : 0;
        auto    blzchrc = reinterpret_cast<Characteristic*>(cbptr);
        auto&   chrc    = blzchrc->chrc;

        std::cout << std::endl
                  << "CCC:Read:UUID:" << (std::string)chrc->GetUUID() << " Handle:" << blzchrc->handle
                  << " Chrc:" << static_cast<void*>(chrc) << " Backend:" << static_cast<void*>(blzchrc) << std::endl;

        uint16_t value = chrc->HasNotifications();

        gatt_db_attribute_read_result(attrib, id, error, (uint8_t*)&value, 2);
    } catch (std::exception const& ex)
    {
        std::cerr << "Error ReadCCCCallback: " << ex.what() << std::endl;
        gatt_db_attribute_read_result(attrib, id, -1, nullptr, 0);
    }

    static void WriteCCCCallback(gatt_db_attribute* attrib,
                                 uint               id,
                                 uint16_t           offset,
                                 const uint8_t*     value,
                                 size_t             len,
                                 uint8_t            opcode,
                                 bt_att*            att,
                                 void*              cbptr) noexcept
    try
    {
        std::unique_lock<std::mutex> lock(globalMutex);

        uint8_t error = 0;

        if (!value || len != 2)
        {
            std::cout << std::endl << "CCC:Write: Invalid Value" << std::endl;
            gatt_db_attribute_write_result(attrib, id, BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN);
            return;
        }
        if (offset > 0)
        {
            std::cout << std::endl << "CCC:Write: Invalid Offset" << std::endl;
            gatt_db_attribute_write_result(attrib, id, BT_ATT_ERROR_INVALID_OFFSET);
            return;
        }

        auto  blzchrc = reinterpret_cast<Characteristic*>(cbptr);
        auto& chrc    = blzchrc->chrc;
        std::cout << std::endl
                  << "CCC:Write:" << (int)value[0] << "UUID:" << (std::string)chrc->GetUUID() << " Handle:" << blzchrc->handle
                  << " Chrc:" << static_cast<void*>(chrc) << " Backend:" << static_cast<void*>(blzchrc) << std::endl;

        if (value[0] == 0x00)
            chrc->DisableNotifications();
        else if (value[0] == 0x01)
            chrc->EnableNotifications();
        else
            error = 0x80;

        /* updating a timer function to call notification on a periodic interval */
        gatt_db_attribute_write_result(attrib, id, error);
    } catch (std::exception const& ex)
    {
        std::cerr << "Error WriteCCCCallback: " << ex.what() << std::endl;
        gatt_db_attribute_write_result(attrib, id, -1);
    }

    static void ReadCallback(gatt_db_attribute* attrib, uint id, uint16_t offset, uint8_t opcode, bt_att* att, void* cbptr)
    {
        try
        {
            std::unique_lock<std::mutex> lock(globalMutex);

            auto  blzchrc = reinterpret_cast<Characteristic*>(cbptr);
            auto& chrc    = blzchrc->chrc;
            std::cout << "Requesting Read for Chrc: " << (std::string)chrc->GetUUID() << " Handle:" << blzchrc->handle
                      << " Chrc:" << static_cast<void*>(chrc) << " Backend:" << static_cast<void*>(blzchrc) << std::endl;
            uint8_t buffer[24];
            auto    len = chrc->ReadValue(buffer);
            gatt_db_attribute_read_result(attrib, id, 0, buffer, len);
        } catch (std::exception const& ex)
        {
            std::cerr << "Error reading Char Value: " << ex.what() << std::endl;
            gatt_db_attribute_read_result(attrib, id, -1, nullptr, 0);
        }
    }

    static void WriteCallback(gatt_db_attribute* attrib,
                              uint               id,
                              uint16_t           offset,
                              const uint8_t*     value,
                              size_t             len,
                              uint8_t            opcode,
                              bt_att*            att,
                              void*              cbptr)

    try
    {
        std::unique_lock<std::mutex> lock(globalMutex);
        auto&                        chrc = reinterpret_cast<Characteristic*>(cbptr)->chrc;
        chrc->WriteValue({value, len});
        gatt_db_attribute_write_result(attrib, id, 0);
    } catch (std::exception const& ex)
    {
        std::cerr << "Error writing Char Value: " << ex.what() << std::endl;
        gatt_db_attribute_write_result(attrib, id, -1);
    }
};

}    // namespace Bluez

void blegatt::ICharacteristic::NotifyUpdated()
{
    if (!_handle.get()) return;
    if (!HasNotifications()) { return; }

    auto chrcbackend = reinterpret_cast<Bluez::Characteristic*>(_handle.get());
    chrcbackend->MarkNotificationPending();
}

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2014  Google Inc.
 *
 *
 */

#define UUID_GAP 0x1800
#define UUID_GATT 0x1801

#define ATT_CID 4

#define PRLOG(...)           \
    do {                     \
        printf(__VA_ARGS__); \
    } while (0)

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define COLOR_OFF "\x1B[0m"
#define COLOR_RED "\x1B[0;91m"
#define COLOR_GREEN "\x1B[0;92m"
#define COLOR_YELLOW "\x1B[0;93m"
#define COLOR_BLUE "\x1B[0;94m"
#define COLOR_MAGENTA "\x1B[0;95m"
#define COLOR_BOLDGRAY "\x1B[1;30m"
#define COLOR_BOLDWHITE "\x1B[1;37m"

static void att_disconnect_cb(int err, void* user_data)
{
    printf("Device disconnected: %s\n", strerror(err));
    restart = true;
    mainloop_quit();
}
#if (defined ATT_DEBUG)
static void att_debug_cb(const char* str, void* user_data)
{
    const char* prefix = reinterpret_cast<const char*>(user_data);
    PRLOG(COLOR_BOLDGRAY "%s" COLOR_BOLDWHITE "%s\n" COLOR_OFF, prefix, str);
}
#endif
static void gatt_debug_cb(const char* str, void* user_data)
{
    const char* prefix = reinterpret_cast<const char*>(user_data);
    PRLOG(COLOR_GREEN "%s%s\n" COLOR_OFF, prefix, str);
}

static void gap_device_name_read_cb(struct gatt_db_attribute* attrib,
                                    unsigned int              id,
                                    uint16_t                  offset,
                                    uint8_t                   opcode,
                                    struct bt_att*            att,
                                    void*                     user_data)
{
    Bluez::Application* server = reinterpret_cast<Bluez::Application*>(user_data);
    uint8_t             error  = 0;
    size_t              len    = 0;
    const uint8_t*      value  = NULL;

    PRLOG("GAP Device Name Read called\n");

    auto deviceName = server->app->Name();
    len             = deviceName.size();

    if (offset > len)
    {
        error = BT_ATT_ERROR_INVALID_OFFSET;
        goto done;
    }

    len -= offset;
    value = len ? reinterpret_cast<const uint8_t*>(&deviceName.data()[offset]) : NULL;

done:
    gatt_db_attribute_read_result(attrib, id, error, value, len);
}

static void gap_device_name_ext_prop_read_cb(struct gatt_db_attribute* attrib,
                                             unsigned int              id,
                                             uint16_t                  offset,
                                             uint8_t                   opcode,
                                             struct bt_att*            att,
                                             void*                     user_data)
{
    uint8_t value[2];

    PRLOG("Device Name Extended Properties Read called\n");

    value[0] = BT_GATT_CHRC_EXT_PROP_RELIABLE_WRITE;
    value[1] = 0;

    gatt_db_attribute_read_result(attrib, id, 0, value, sizeof(value));
}

static void gatt_service_changed_cb(struct gatt_db_attribute* attrib,
                                    unsigned int              id,
                                    uint16_t                  offset,
                                    uint8_t                   opcode,
                                    struct bt_att*            att,
                                    void*                     user_data)
{
    PRLOG("Service Changed Read called\n");

    gatt_db_attribute_read_result(attrib, id, 0, NULL, 0);
}

static void gatt_svc_chngd_ccc_read_cb(struct gatt_db_attribute* attrib,
                                       unsigned int              id,
                                       uint16_t                  offset,
                                       uint8_t                   opcode,
                                       struct bt_att*            att,
                                       void*                     user_data)
{
    Bluez::Application* server = reinterpret_cast<Bluez::Application*>(user_data);

    uint8_t value[2];

    PRLOG("Service Changed CCC Read called\n");

    value[0] = server->svc_chngd_enabled ? 0x02 : 0x00;
    value[1] = 0x00;

    gatt_db_attribute_read_result(attrib, id, 0, value, sizeof(value));
}

static void gatt_svc_chngd_ccc_write_cb(struct gatt_db_attribute* attrib,
                                        unsigned int              id,
                                        uint16_t                  offset,
                                        const uint8_t*            value,
                                        size_t                    len,
                                        uint8_t                   opcode,
                                        struct bt_att*            att,
                                        void*                     user_data)
{
    Bluez::Application* server = reinterpret_cast<Bluez::Application*>(user_data);
    uint8_t             ecode  = 0;

    PRLOG("Service Changed CCC Write called\n");

    if (!value || len != 2)
    {
        ecode = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
        goto done;
    }

    if (offset)
    {
        ecode = BT_ATT_ERROR_INVALID_OFFSET;
        goto done;
    }

    if (value[0] == 0x00)
        server->svc_chngd_enabled = false;
    else if (value[0] == 0x02)
        server->svc_chngd_enabled = true;
    else
        ecode = 0x80;

    PRLOG("Service Changed Enabled: %s\n", server->svc_chngd_enabled ? "true" : "false");

done:
    gatt_db_attribute_write_result(attrib, id, ecode);
}

static void confirm_write(struct gatt_db_attribute* attr, int err, void* user_data)
{
    if (!err) return;

    fprintf(stderr, "Error caching attribute %p - err: %d\n", attr, err);
    exit(1);
}

static void populate_gap_service(Bluez::Application* server)
{
    bt_uuid_t                 uuid;
    struct gatt_db_attribute *service, *tmp;
    uint16_t                  appearance;

    /* Add the GAP service */
    bt_uuid16_create(&uuid, UUID_GAP);
    service = check_not_null(gatt_db_add_service(server->db, &uuid, true, 6), "Failed to add GAP service");

    /*
     * Device Name characteristic. Make the value dynamically read and
     * written via callbacks.
     */
    bt_uuid16_create(&uuid, GATT_CHARAC_DEVICE_NAME);
    check_not_null(gatt_db_service_add_characteristic(service,
                                                      &uuid,
                                                      BT_ATT_PERM_READ,
                                                      BT_GATT_CHRC_PROP_READ | BT_GATT_CHRC_PROP_EXT_PROP,
                                                      gap_device_name_read_cb,
                                                      nullptr,
                                                      server),
                   "Failed to Add Device Name chrc to GAP service");

    bt_uuid16_create(&uuid, GATT_CHARAC_EXT_PROPER_UUID);
    check_not_null(gatt_db_service_add_descriptor(service, &uuid, BT_ATT_PERM_READ, gap_device_name_ext_prop_read_cb, NULL, server),
                   "Faied to add ext_proper chrc to GAP service");

    /*
     * Appearance characteristic. Reads and writes should obtain the value
     * from the database.
     */
    bt_uuid16_create(&uuid, GATT_CHARAC_APPEARANCE);
    tmp = check_not_null(gatt_db_service_add_characteristic(service, &uuid, BT_ATT_PERM_READ, BT_GATT_CHRC_PROP_READ, NULL, NULL, server),
                         "Faied to add Appearance chrc to GAP service");

    /*
     * Write the appearance value to the database, since we're not using a
     * callback.
     */
    put_le16(128, &appearance);
    gatt_db_attribute_write(tmp, 0, (uint8_t*)&appearance, sizeof(appearance), BT_ATT_OP_WRITE_REQ, NULL, confirm_write, NULL);

    gatt_db_service_set_active(service, true);
}

static void populate_gatt_service(Bluez::Application* server)
{
    bt_uuid_t                 uuid;
    struct gatt_db_attribute *service, *svc_chngd;

    /* Add the GATT service */
    bt_uuid16_create(&uuid, UUID_GATT);
    service = check_not_null(gatt_db_add_service(server->db, &uuid, true, 4), "Failed to add GATT service");

    bt_uuid16_create(&uuid, GATT_CHARAC_SERVICE_CHANGED);
    svc_chngd = check_not_null(
        gatt_db_service_add_characteristic(
            service, &uuid, BT_ATT_PERM_READ, BT_GATT_CHRC_PROP_READ | BT_GATT_CHRC_PROP_INDICATE, gatt_service_changed_cb, NULL, server),
        "Failed to Add Svc-Changed chrc to GATT service");

    server->gatt_svc_chngd_handle = gatt_db_attribute_get_handle(svc_chngd);

    bt_uuid16_create(&uuid, GATT_CLIENT_CHARAC_CFG_UUID);
    check_not_null(
        gatt_db_service_add_descriptor(
            service, &uuid, BT_ATT_PERM_READ | BT_ATT_PERM_WRITE, gatt_svc_chngd_ccc_read_cb, gatt_svc_chngd_ccc_write_cb, server),
        "Failed to Chrc Config Chrc to Descp to Chrc");

    gatt_db_service_set_active(service, true);
}

Bluez::Application::Application(int index)
{
    adapterIndex = index;
    check_rc(hci_devba(index, &addr), "Cannot find hci0");
    char ba[18];
    ba2str(&addr, ba);
    printf("Local: %s\n", ba);
    addr_type = BDADDR_LE_PUBLIC;
    db        = check_not_null(gatt_db_new(), "Failed to create GATT database");
    mgmt      = check_not_null(mgmt_new_default(), "Failed to access management interface");
}

void Bluez::Application::Clear()
{
    auto that = app;
    for (size_t i = 0; i < that->ServiceCount(); i++)
    {
        auto& svc = that->ServiceAt(i);
        svc._handle.reset(nullptr);
        for (size_t j = 0; j < svc.CharacteristicsCount(); j++)
        {
            auto& chrc = svc.CharacteristicAt(j);
            chrc._handle.reset(nullptr);
        }
    }
    that->_handle.reset(nullptr);
}

void Bluez::Application::Init(int fd, uint16_t mtu)
{
    std::unique_lock<std::mutex>      lock(globalMutex);
    std::unordered_set<blegatt::UUID> uuids;
    att = check_not_null(bt_att_new(fd, false), "Failed to initialze ATT transport layer");
    if (!bt_att_set_close_on_unref(att, true)) throw std::runtime_error("Failed to set up ATT transport layer\n");
    if (!bt_att_register_disconnect(att, att_disconnect_cb, NULL, NULL)) throw std::runtime_error("Failed to set ATT disconnect handler\n");

    gatt = check_not_null(bt_gatt_server_new(db, att, mtu, 0), "Failed to create GATT server");
#if (defined ATT_DEBUG)
    bt_att_set_debug(att, att_debug_cb, (void*)"att: ", NULL);
#endif
    bt_gatt_server_set_debug(gatt, gatt_debug_cb, (void*)"server: ", NULL);

    populate_gap_service(this);
    populate_gatt_service(this);
    auto that = this->app;
    for (size_t i = 0; i < that->ServiceCount(); i++)
    {
        auto& svc     = that->ServiceAt(i);
        auto  svcuuid = svc.GetUUID();
        if (!uuids.insert(svcuuid).second) { throw std::logic_error("Duplication Svc UUID found"); }
        else
        {
            std::cout << "Register Service: " << (std::string)svcuuid << std::endl;
        }
        auto blzsvc = new Bluez::Service();
        blzsvc->app = this;
        blzsvc->svc = &svc;
        blzsvc->attr
            = check_not_null(gatt_db_add_service(db, &svcuuid, true, 8 /* TODO: Figure out the count */), "Cannot add Application Service");
        blzsvc->handle = gatt_db_attribute_get_handle(blzsvc->attr);
        svc._handle.reset(blzsvc);
        for (size_t j = 0; j < svc.CharacteristicsCount(); j++)
        {
            auto& chrc     = svc.CharacteristicAt(j);
            auto  chrcuuid = chrc.GetUUID();
            if (!uuids.insert(chrcuuid).second) { throw std::logic_error("Duplication Chrc UUID found"); }
            uint8_t properties = 0;
            if (chrc.AllowNotification()) properties |= BT_GATT_CHRC_PROP_NOTIFY;
            if (chrc.IsReadSupported()) properties |= BT_GATT_CHRC_PROP_READ;
            if (chrc.IsWriteSupported()) properties |= BT_GATT_CHRC_PROP_READ;

            auto blzchrc  = new Bluez::Characteristic();
            blzchrc->svc  = blzsvc;
            blzchrc->app  = this;
            blzchrc->chrc = &chrc;
            blzchrc->attr = check_not_null(gatt_db_service_add_characteristic(blzsvc->attr,
                                                                              &chrcuuid,
                                                                              BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
                                                                              properties,
                                                                              Bluez::Characteristic::ReadCallback,
                                                                              Bluez::Characteristic::WriteCallback,
                                                                              blzchrc),
                                           "Add Characteristic");

            blzchrc->handle = gatt_db_attribute_get_handle(blzchrc->attr);
            if (chrc.AllowNotification())
            {
                bt_uuid_t cccuuid;
                bt_uuid16_create(&cccuuid, GATT_CLIENT_CHARAC_CFG_UUID);
                check_not_null(gatt_db_service_add_descriptor(blzsvc->attr,
                                                              &cccuuid,
                                                              BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
                                                              Bluez::Characteristic::ReadCCCCallback,
                                                              Bluez::Characteristic::WriteCCCCallback,
                                                              blzchrc),
                               "Cannot add Chrc Config Descriptor to Chrc");
            }

            std::cout << "Register Characteristic: " << (std::string)chrcuuid << " Handle:" << blzchrc->handle
                      << " Chrc:" << static_cast<void*>(&chrc) << " Backend:" << static_cast<void*>(blzchrc) << std::endl;

            chrc._handle.reset(blzchrc);
        }
        gatt_db_service_set_active(blzsvc->attr, true);
    }
}

static void mgmt_generic_callback_complete(uint8_t status, uint16_t length, const void* param, void* user_data)
{
    if (status != MGMT_STATUS_SUCCESS)
    {
        fprintf(stderr, "%s failed: %s\n", (char*)user_data, mgmt_errstr(status));
        return;
    }

    printf("%s completed\n", (char*)user_data);
}

static void mgmt_generic_event(uint16_t index, uint16_t length, const void* param, void* user_data)
{
    printf("%s\n", (char*)user_data);
}

static std::vector<uint8_t> bt_ad_generate_data(Bluez::Application* server)
{
    struct bt_ad* data = check_not_null(bt_ad_new(), "Error creating adverting data\n");

    for (size_t i = 0; i < server->app->ServiceCount(); i++)
    {
        auto& svc = server->app->ServiceAt(i);
        if (!svc.Advertise()) continue;
        auto uuid = svc.GetUUID();
        check_rc(bt_ad_add_service_uuid(data, &uuid), "Error adding service UUID\n");
    }

    size_t adv_data_len;
    auto   adv_data = check_not_null(bt_ad_generate(data, &adv_data_len), "Error generating advertising data");
    bt_ad_unref(data);
    std::vector<uint8_t> rslt;
    rslt.reserve(adv_data_len);
    for (size_t i = 0; i < adv_data_len; i++) { rslt.push_back(adv_data[i]); }
    free(adv_data);
    return rslt;
}
static void advertise(Bluez::Application* server)
{
    uint8_t  load_conn_params_len;
    uint8_t  add_adv_len;
    uint32_t flags;

    auto    adv_data     = bt_ad_generate_data(server);
    auto    adv_data_len = adv_data.size();
    uint8_t val          = 0x00;
    check_rc(mgmt_send(server->mgmt,
                       MGMT_OP_SET_POWERED,
                       server->adapterIndex,
                       1,
                       &val,
                       mgmt_generic_callback_complete,
                       (void*)"MGMT_OP_SET_POWERED",
                       NULL),
             "Failed setting powered off");
    load_conn_params_len = sizeof(struct mgmt_cp_load_conn_param) + sizeof(struct mgmt_conn_param);
    CMem<struct mgmt_cp_load_conn_param> load_conn_params([&]() { return malloc0(load_conn_params_len); });
    CMem<struct mgmt_conn_param>         conn_param([&]() { return malloc0(sizeof(struct mgmt_conn_param)); });

    bacpy(&conn_param->addr.bdaddr, &server->addr);
    conn_param->addr.type         = server->addr_type;
    conn_param->min_interval      = 6;
    conn_param->max_interval      = 12;
    conn_param->latency           = 0;
    conn_param->timeout           = 200;
    load_conn_params->param_count = 1;
    memcpy(load_conn_params->params, conn_param, sizeof(struct mgmt_conn_param));

    check_rc(mgmt_send(server->mgmt,
                       MGMT_OP_LOAD_CONN_PARAM,
                       server->adapterIndex,
                       load_conn_params_len,
                       load_conn_params,
                       mgmt_generic_callback_complete,
                       (void*)"MGMT_OP_LOAD_CONN_PARAM",
                       NULL),
             "Failed to load connection parameters\n");

    CMem<struct mgmt_cp_set_local_name> localname([&]() { return malloc0(sizeof(struct mgmt_cp_set_local_name)); });

    auto   name = server->app->Name();
    size_t i    = 0;
    for (i = 0; i < name.size() && i < std::size(localname->name) - 1 && i < std::size(localname->short_name) - 1; i++)
    {
        localname->short_name[i] = localname->name[i] = static_cast<uint8_t>(name[i]);
    }
    localname->name[i]       = 0;
    localname->short_name[i] = 0;

    check_rc(mgmt_send(server->mgmt,
                       MGMT_OP_SET_LOCAL_NAME,
                       server->adapterIndex,
                       sizeof(struct mgmt_cp_set_local_name),
                       localname,
                       mgmt_generic_callback_complete,
                       (void*)"MGMT_OP_SET_LOCAL_NAME",
                       NULL),
             "Failed setting local name\n");

    val = 0x01;
    check_rc(
        mgmt_send(
            server->mgmt, MGMT_OP_SET_LE, server->adapterIndex, 1, &val, mgmt_generic_callback_complete, (void*)"MGMT_OP_SET_LE", NULL),
        "Failed setting low energy\n");

    val = 0x01;
    check_rc(mgmt_send(server->mgmt,
                       MGMT_OP_SET_CONNECTABLE,
                       server->adapterIndex,
                       1,
                       &val,
                       mgmt_generic_callback_complete,
                       (void*)"MGMT_OP_SET_CONNECTABLE",
                       NULL),
             "Failed setting connectable\n");

    add_adv_len = sizeof(struct mgmt_cp_add_advertising) + adv_data_len;
    CMem<struct mgmt_cp_add_advertising> add_adv([&]() { return malloc0(add_adv_len); });
    // struct mgmt_cp_add_advertising*      add_adv = add_adv_tracker.ptr;

    flags                 = MGMT_ADV_FLAG_CONNECTABLE | MGMT_ADV_FLAG_DISCOV;
    add_adv->instance     = 1;
    add_adv->flags        = htobl(flags);
    add_adv->duration     = 0;
    add_adv->timeout      = 0;
    add_adv->adv_data_len = adv_data_len;
    add_adv->scan_rsp_len = 0;
    memcpy(add_adv->data, adv_data.data(), adv_data_len);

    check_rc(mgmt_send(server->mgmt,
                       MGMT_OP_ADD_ADVERTISING,
                       server->adapterIndex,
                       add_adv_len,
                       add_adv,
                       mgmt_generic_callback_complete,
                       (void*)"MGMT_OP_ADD_ADVERTISING",
                       NULL),
             "Failed to add advertising");

    mgmt_register(
        server->mgmt, MGMT_EV_DEVICE_CONNECTED, server->adapterIndex, mgmt_generic_event, (void*)"MGMT_EV_DEVICE_CONNECTED", NULL);

    mgmt_register(
        server->mgmt, MGMT_EV_DEVICE_DISCONNECTED, server->adapterIndex, mgmt_generic_event, (void*)"MGMT_EV_DEVICE_DISCONNECTED", NULL);

    val = 0x01;
    check_rc(mgmt_send(server->mgmt,
                       MGMT_OP_SET_POWERED,
                       server->adapterIndex,
                       1,
                       &val,
                       mgmt_generic_callback_complete,
                       (void*)"MGMT_OP_SET_POWERED",
                       NULL),
             "Failed setting powered on");
}

static void att_conn_callback(int fd, uint32_t events, void* user_data)
{
    Bluez::Application* server = reinterpret_cast<Bluez::Application*>(user_data);

    int                new_fd;
    struct sockaddr_l2 addr;
    socklen_t          optlen;
    char               ba[18];
    uint16_t           mtu = 0;

    memset(&addr, 0, sizeof(addr));
    optlen = sizeof(addr);
    new_fd = accept(fd, (struct sockaddr*)&addr, &optlen);
    if (new_fd < 0)
    {
        perror("Accept failed");
        return;
    }

    ba2str(&addr.l2_bdaddr, ba);
    printf("Connect from %s\n", ba);

    server->Init(new_fd, mtu);
    std::cout << "Server Initialized" << std::endl;
}

static int l2cap_le_att_listen(bdaddr_t* src, uint8_t sec, uint8_t src_type)
{
    int                sk;
    struct sockaddr_l2 srcaddr;
    struct bt_security btsec;

    sk = check_rc(socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP), "Failed to create L2CAP socket");

    /* Set up source address */
    memset(&srcaddr, 0, sizeof(srcaddr));
    srcaddr.l2_family      = AF_BLUETOOTH;
    srcaddr.l2_cid         = htobs(ATT_CID);
    srcaddr.l2_bdaddr_type = src_type;
    bacpy(&srcaddr.l2_bdaddr, src);

    check_rc(bind(sk, (struct sockaddr*)&srcaddr, sizeof(srcaddr)), "Failed to bind L2CAP socket");

    /* Set the security level */
    memset(&btsec, 0, sizeof(btsec));
    btsec.level = sec;
    check_rc(setsockopt(sk, SOL_BLUETOOTH, BT_SECURITY, &btsec, sizeof(btsec)), "Failed to set L2CAP security level\n");
    check_rc(listen(sk, 10), "Listening on socket failed");
    return sk;
}

void blegatt::IApplication::Start()
{
    _thrd = std::jthread([this](std::stop_token stoken) {
        while (!stoken.stop_requested()) _StartImpl();
    });
}

blegatt::IApplication::IApplication()
{
    auto server = new Bluez::Application(0);
    server->app = this;
    this->_handle.reset(server);
}

blegatt::IApplication::~IApplication()
{}

void blegatt::IApplication::Stop()
{
    _thrd.request_stop();
    mainloop_quit();
    _thrd.join();
}

void blegatt::IApplication::_StartImpl()
{
    auto server = reinterpret_cast<Bluez::Application*>(this->_handle.get());
    std::cout << "Start BLE Application" << std::endl;
    uint8_t src_type = BDADDR_LE_PUBLIC;
    mainloop_init();
    FileDescriptor fd(l2cap_le_att_listen(&server->addr, BT_SECURITY_LOW, src_type));
    check_rc(mainloop_add_fd(fd, EPOLLIN, att_conn_callback, server, NULL), "Erro adding connection callback to mainloop");
    advertise(server);
#if 0
    AdvertisementManager adv(0, server->mgmt);
    
    for (size_t i = 0; i < server->app->ServiceCount(); i++)
    {
        auto& svc = server->app->ServiceAt(i);
        if (!svc.Advertise()) continue;
        adv.AddServiceUUID(svc.GetUUID());
    }
    adv.Refresh();
#endif
    printf("Running GATT server\n");

    mainloop_run();
    {
        std::unique_lock<std::mutex> lock(globalMutex);
        server->Clear();
        this->_handle.reset(nullptr);
        // mainloop_quit();
        printf("\n\nShutting down...\n");
    }
}
