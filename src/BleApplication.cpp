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
extern "C"
{
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <glib.h>

#include <dbus/dbus.h>

#include "lib/bluetooth.h"

#include "gdbus/gdbus.h"
#define class deviceclass

#include "backtrace.h"
#include "hcid.h"
#include "lib/uuid.h"
#include "log.h"
#include "shared/ad.h"
#include "shared/att-types.h"
#include "shared/gatt-db.h"
#include "shared/mainloop.h"
#include "shared/mgmt.h"
#include "shared/util.h"

#include "adapter.h"
#include "agent.h"
#include "dbus-common.h"
#include "gatt-database.h"

#include "device.h"
#include "mgmt.h"
#include "profile.h"
#undef class
}

#define DEFAULT_PAIRABLE_TIMEOUT 0       /* disabled */
#define DEFAULT_DISCOVERABLE_TIMEOUT 180 /* 3 minutes */
#define DEFAULT_TEMPORARY_TIMEOUT 30     /* 30 seconds */

#define SHUTDOWN_GRACE_SECONDS 10
#include "BleApplication.h"

#include <future>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <unordered_map>

struct main_opts main_opts;

struct FreeDeleter
{
    void operator()(void* ptr) const { free(ptr); }
};

template <typename T> struct CMem
{
    template <typename TFunc> CMem(TFunc&& f)
    {
        ptr = f(len);
        if (ptr == nullptr || len == 0)
        {
            throw std::runtime_error("Empty Return");
        }
    }
    ~CMem() { free(ptr); }

    T*     ptr{};
    size_t len{0};
};

void btd_exit(void)
{
    mainloop_quit();
}

struct AdvertisementManager
{
    enum class Type
    {
        BroadCast  = 0,
        Peripheral = 1
    };

    static AdvertisementManager& Get(uint16_t index)
    {
        _created.get_future().wait();
        return *_managers[index];
    }

    AdvertisementManager(struct btd_adapter* adapter, struct mgmt* mgmt) : _adapter(adapter), _mgmt(mgmt)
    {
        if (adapter == nullptr) throw std::invalid_argument("Adapter null");
        if (mgmt == nullptr) throw std::invalid_argument("Mgmt null");
        mgmt_ref(mgmt);
        btd_adapter_ref(adapter);
        _adapterIndex = btd_adapter_get_index(adapter);
        _adapterName  = btd_adapter_get_name(adapter);
        uint8_t val   = 0x01;
        if (!mgmt_send(mgmt, MGMT_OP_SET_POWERED, _adapterIndex, 1, &val, _AddAdvComplete, this, nullptr))
        {
            throw std::runtime_error("Unable to turn it on");
        }

        if (!mgmt_send(mgmt, MGMT_OP_READ_ADV_FEATURES, _adapterIndex, 0, NULL, _AdvFeatureReadComplete, this, nullptr))
        {
            throw std::runtime_error("Failed to read advertising features");
        }

        _managers[_adapterIndex] = this;
        _created.set_value();
    }

    void _AcquireSlot()
    {
        std::unique_lock<std::mutex> lock(_mutex);
        std::swap(_lock, lock);
    }

    void _ReleaseSlot() { _lock.release(); }

    ~AdvertisementManager()
    {
        _managers.erase(_adapterIndex);
        _AcquireSlot();
        _ReleaseSlot();
        _init.get_future().wait();
        mgmt_unref(_mgmt);
        btd_adapter_unref(_adapter);
    }

    void _InitComplete(mgmt_rp_read_adv_features const& features)
    {
        _features = features;
        _supportedFlags |= _features.supported_flags;
        _init.set_value();
    }

    static void _AdvFeatureReadComplete(uint8_t status, uint16_t length, const void* featp, void* thatp)
    {
        auto that = reinterpret_cast<AdvertisementManager*>(thatp);
        auto feat = reinterpret_cast<mgmt_rp_read_adv_features const*>(featp);
        try
        {
            if (status || !feat)
            {
                that->_init.set_exception(std::exception_ptr());
                return;
            }

            if (length < sizeof(*feat))
            {
                that->_init.set_exception(std::exception_ptr());
                return;
            }
            that->_InitComplete(*feat);
        }
        catch (...)
        {
            that->_init.set_exception(std::current_exception());
        }
    }

    static size_t calc_max_adv_len(uint8_t max, uint32_t flags)
    {
        /*
         * Flags which reduce the amount of space available for advertising.
         * See doc/mgmt-api.txt
         */
        if (flags & MGMT_ADV_FLAG_TX_POWER) max -= 3;

        if (flags & (MGMT_ADV_FLAG_DISCOV | MGMT_ADV_FLAG_LIMITED_DISCOV | MGMT_ADV_FLAG_MANAGED_FLAGS)) max -= 3;

        if (flags & MGMT_ADV_FLAG_APPEARANCE) max -= 4;

        return max;
    }

    static void _AddAdvComplete(uint8_t status, uint16_t length, const void* featp, void* thatp)
    {
        if (status)
        {
            error("Advertisement Refresh Failed with  status code : %d", status);
        }
        else
        {
            info("Advertisement Refreshed");
        }
    }

    void AddServiceUUID(std::string_view str)
    {
        bt_uuid_t uuid;
        bt_string_to_uuid(&uuid, str.data());
        AddServiceUUID(uuid);
    }
    void AddServiceUUID(bt_uuid_t const& uuid) { bt_ad_add_service_uuid(_client.data.get(), &uuid); }
    void Reset()
    {
        AdvClient client;
        std::swap(_client, client);
        Refresh();
    }

    void Refresh()
    {
        uint32_t flags = 0;

        if (_client.type == Type::Peripheral)
        {
            flags |= MGMT_ADV_FLAG_CONNECTABLE;

            if (btd_adapter_get_discoverable(_adapter) && !(bt_ad_has_flags(_client.data.get()))) flags |= MGMT_ADV_FLAG_DISCOV;
        }

        // flags |= _client.flags;

        // bt_ad_add_appearance(_client.data.get(), _client.appearance);

        auto adv = CMem<uint8_t>([&](size_t& len) { return bt_ad_generate(_client.data.get(), &len); });

        if (adv.len > calc_max_adv_len(_features.max_adv_data_len, flags))
        {
            throw std::runtime_error("Advertising data too long or couldn't be generated.");
        }

        // flags &= ~MGMT_ADV_FLAG_LOCAL_NAME;
        // flags |= MGMT_ADV_FLAG_LOCAL_NAME
        bt_ad_add_name(_client.scan.get(), _client.name ? _client.name : _adapterName.data());
        auto scanRSP = CMem<uint8_t>([&](size_t& len) { return bt_ad_generate(_client.scan.get(), &len); });

        std::unique_ptr<mgmt_cp_add_advertising, FreeDeleter> cpptr(
            reinterpret_cast<mgmt_cp_add_advertising*>(malloc(sizeof(mgmt_cp_add_advertising) + adv.len + scanRSP.len)));

        auto& cp = *cpptr;

        uint8_t len     = sizeof(mgmt_cp_add_advertising) + adv.len + scanRSP.len;
        cp.flags        = htobl(flags);
        cp.instance     = _client.instance + 1;
        cp.duration     = _client.duration;
        cp.adv_data_len = adv.len;
        cp.scan_rsp_len = scanRSP.len;
        memcpy(cp.data, adv.ptr, adv.len);
        memcpy(cp.data + adv.len, scanRSP.ptr, scanRSP.len);

        if (!mgmt_send(_mgmt, MGMT_OP_ADD_ADVERTISING, _adapterIndex, len, &cp, _AddAdvComplete, nullptr, nullptr))
        {
            throw std::runtime_error("Failed to add Advertising Data");
        }
    }

    /*
            { "Type", parse_type },
    { "ServiceUUIDs", parse_service_uuids },
        { "SolicitUUIDs", parse_solicit_uuids },
        { "ManufacturerData", parse_manufacturer_data },
        { "ServiceData", parse_service_data },
        { "Includes", parse_includes },
        { "LocalName", parse_local_name },
        { "Appearance", parse_appearance },
        { "Duration", parse_duration },
        { "Timeout", parse_timeout },
        { "Data", parse_data },
        { "Discoverable", parse_discoverable },
        { "DiscoverableTimeout", parse_discoverable_timeout },
        { "SecondaryChannel", parse_secondary },
        */
    struct AdvClient
    {
        struct AdDeleter
        {
            void operator()(struct bt_ad* ad) { bt_ad_unref(ad); }
        };

        using BTAd = std::unique_ptr<struct bt_ad, AdDeleter>;
        BTAd data{bt_ad_new()};
        BTAd scan{bt_ad_new()};

        uint32_t    flags{0};
        uint8_t     instance{0};
        uint16_t    duration{0};
        const char* name{nullptr};
        Type        type{Type::Peripheral};
        uint8_t     appearance{0};
    } _client;

    std::unique_lock<std::mutex> _lock;
    std::mutex                   _mutex;
    std::promise<void>           _init;
    uint32_t                     _supportedFlags{MGMT_ADV_FLAG_LOCAL_NAME};
    mgmt_rp_read_adv_features    _features{};
    uint16_t                     _adapterIndex{0};
    btd_adapter*                 _adapter{};
    mgmt*                        _mgmt{};
    std::string_view             _adapterName;

    static inline std::promise<void>                                  _created;
    static inline std::unordered_map<uint16_t, AdvertisementManager*> _managers;
};

extern "C" void* btd_adv_manager_new(struct btd_adapter* adapter, struct mgmt* mgmt)
try
{
    DBG("LE Advertising Manager created for adapter: %s", adapter_get_path(adapter));
    return new AdvertisementManager(adapter, mgmt);
}
catch (std::exception const& ex)
{
    DBG("Error Creating LE Advertisement Manager : %s", ex.what());
    return nullptr;
}

extern "C" void btd_adv_manager_destroy(AdvertisementManager* manager)
{
    delete reinterpret_cast<AdvertisementManager*>(manager);
}

extern "C" void btd_adv_manager_refresh(AdvertisementManager* manager)
{
    manager->Refresh();
}

struct ChrcBackend
{
    blegatt::ICharacteristic* chrc;
};

namespace Bluez
{
struct Service : blegatt::IBackendHandler
{
    blegatt::IService* svc{nullptr};
    uint16_t           handle{};
    gatt_db_attribute* attr{nullptr};
};

struct Characteristic : blegatt::IBackendHandler
{
    blegatt::ICharacteristic* chrc{nullptr};
    uint16_t                  handle{};
    struct gatt_db_attribute* attr{nullptr};

    static void ReadCallback(gatt_db_attribute* attrib, uint id, uint16_t offset, uint8_t opcode, bt_att* att, void* cbptr)
    {
        auto& chrc = reinterpret_cast<Characteristic*>(cbptr)->chrc;

        uint8_t buffer[24];
        auto    len = chrc->ReadValue(buffer);
        gatt_db_attribute_read_result(attrib, id, 0, buffer, len);
    }

    static void WriteCallback(gatt_db_attribute* attrib,
                              uint               id,
                              uint16_t           offset,
                              const uint8_t*     value,
                              size_t             len,
                              uint8_t            opcode,
                              bt_att*            att,
                              void*              cbptr)
    {
        auto& chrc = reinterpret_cast<Characteristic*>(cbptr)->chrc;
        chrc->WriteValue({value, len});
    }
};
}    // namespace Bluez

static void register_svcs_on_adapter(btd_adapter* adapter, void* ptr)
{
    auto that = reinterpret_cast<blegatt::IApplication*>(ptr);
    auto db   = btd_gatt_database_get_db(btd_adapter_get_database(adapter));
    for (size_t i = 0; i < that->ServiceCount(); i++)
    {
        auto& svc     = that->ServiceAt(i);
        auto  svcuuid = svc.GetUUID();

        /* Add Heart Rate Service */
        auto blzsvc    = new Bluez::Service();
        blzsvc->svc    = &svc;
        blzsvc->attr   = gatt_db_add_service(db, &svcuuid, true, 8 /* TODO: Figure out the count */);
        blzsvc->handle = gatt_db_attribute_get_handle(blzsvc->attr);
        svc._handle.reset(blzsvc);
        for (size_t j = 0; j < svc.CharacteristicsCount(); j++)
        {
            auto&   chrc       = svc.CharacteristicAt(j);
            auto    chrcuuid   = chrc.GetUUID();
            uint8_t properties = 0;
            if (chrc.AllowNotification()) properties |= BT_GATT_CHRC_PROP_NOTIFY;
            if (chrc.IsReadSupported()) properties |= BT_GATT_CHRC_PROP_READ;
            if (chrc.IsWriteSupported()) properties |= BT_GATT_CHRC_PROP_READ;

            auto blzchrc  = new Bluez::Characteristic();
            blzchrc->chrc = &chrc;
            blzchrc->attr = gatt_db_service_add_characteristic(blzsvc->attr,
                                                               &chrcuuid,
                                                               BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
                                                               properties,
                                                               Bluez::Characteristic::ReadCallback,
                                                               Bluez::Characteristic::WriteCallback,
                                                               blzchrc);

            blzchrc->handle = gatt_db_attribute_get_handle(blzchrc->attr);
            chrc._handle.reset(blzchrc);
        }
        gatt_db_service_set_active(blzsvc->attr, true);
    }
}

static void register_svcs(blegatt::IApplication* that)
{
    adapter_foreach(register_svcs_on_adapter, that);
}

void blegatt::IApplication::Start()
{
    /*
       [General]
       Name = AViD
       Class = 0x010500
       DiscoverableTimeout = 0
       PairableTimeout = 0
       ControllerMode = le

       [GATT]

       [Policy]
       AutoEnable=true
       */

    // Policy : AutoEnable = true
    // btd_adapter_restore_powered(adapter);

    uint8_t major = 1, minor = 0;

    /* Default HCId settings */
    memset(&main_opts, 0, sizeof(main_opts));
    main_opts.name              = strdup(Name().data());
    main_opts.deviceclass       = 0x010500;
    main_opts.pairto            = 0;    // DEFAULT_PAIRABLE_TIMEOUT;
    main_opts.discovto          = 0;    // DEFAULT_DISCOVERABLE_TIMEOUT;
    main_opts.tmpto             = DEFAULT_TEMPORARY_TIMEOUT;
    main_opts.reverse_discovery = TRUE;
    main_opts.name_resolv       = TRUE;
    main_opts.debug_keys        = FALSE;
    main_opts.refresh_discovery = TRUE;
    main_opts.mode              = BT_MODE_LE;    // BT_MODE_BREDR BT_MODE_DUAL

    main_opts.default_params.num_entries       = 0;
    main_opts.default_params.br_page_scan_type = 0xFFFF;
    main_opts.default_params.br_scan_type      = 0xFFFF;

    main_opts.did_source  = 0x0002; /* USB */
    main_opts.did_vendor  = 0x1d6b; /* Linux Foundation */
    main_opts.did_product = 0x0246; /* BlueZ */
    main_opts.did_version = (major << 8 | minor);

    main_opts.gatt_cache    = BT_GATT_CACHE_ALWAYS;
    main_opts.gatt_mtu      = BT_ATT_MAX_LE_MTU;
    main_opts.gatt_channels = 3;

    umask(0077);

    btd_backtrace_init();

    mainloop_init();

    __btd_log_init("*" /* debug */, 0 /* detach */);

    mainloop_sd_notify("STATUS=Starting up");

    if (adapter_init() < 0)
    {
        throw std::runtime_error("Adapter handling initialization failed");
        exit(1);
    }

    btd_device_init();
    btd_agent_init();
    btd_profile_init();

    rfkill_init();

    DBG("Entering main loop");
    auto ftr = std::async([this]() {
        auto& mgr = AdvertisementManager::Get(0);
        register_svcs(this);

        for (size_t i = 0; i < ServiceCount(); i++)
        {
            auto& svc = ServiceAt(i);
            if (svc.Advertise())
            {
                mgr.AddServiceUUID(svc.GetUUID());
            }
        }
        mgr.Refresh();
    });
    mainloop_sd_notify("STATUS=Running");
    mainloop_sd_notify("READY=1");

    mainloop_run();

    mainloop_sd_notify("STATUS=Quitting");

    plugin_cleanup();

    btd_profile_cleanup();
    btd_agent_cleanup();
    btd_device_cleanup();

    adapter_cleanup();

    rfkill_exit();
    info("Exit");

    __btd_log_cleanup();
}
