#pragma once
#include <iostream>
#include <memory>
#include <sdp.h>
#include <span>
#include <string_view>
#include <uuid.h>

namespace blegatt
{

struct IBackendHandler
{
    virtual ~IBackendHandler() = default;
};

struct UUID : bt_uuid_t
{
    explicit UUID(uint16_t val) { bt_uuid16_create(this, val); }
    UUID(std::string_view str) { bt_string_to_uuid(this, str.data()); }
    UUID(char const* const str) { bt_string_to_uuid(this, str); }

    bool operator==(UUID const& r) const { return bt_uuid_cmp(this, &r) == 0; }

    operator std::string() const
    {
        char buffer[40];
        bt_uuid_to_string(this, buffer, std::size(buffer));
        return std::string(buffer);
    }
};

struct ICharacteristic
{
    virtual UUID GetUUID() const = 0;

    virtual bool IsReadSupported() const   = 0;
    virtual bool IsWriteSupported() const  = 0;
    virtual bool AllowNotification() const = 0;

    virtual size_t ReadValue(std::span<uint8_t> buffer)      = 0;
    virtual void   WriteValue(std::span<const uint8_t> data) = 0;

    void NotifyUpdated();
    bool HasNotifications() { return _notificationsEnabled; }
    void EnableNotifications()
    {
        _notificationsEnabled = true;
        std::cout << "Enabling Notifications on: " << (std::string)GetUUID() << std::endl;
    }

    void DisableNotifications()
    {
        _notificationsEnabled = false;
        std::cout << "Disabling Notifications on: " << (std::string)GetUUID() << std::endl;
    }

    bool                             _notificationsEnabled{false};
    std::unique_ptr<IBackendHandler> _handle;
};

struct IRWNotifyCharacteristic : ICharacteristic
{
    virtual bool IsReadSupported() const override { return true; }
    virtual bool IsWriteSupported() const override { return true; }
    virtual bool AllowNotification() const override { return true; }
};

struct IReadNotifyCharacteristic : ICharacteristic
{
    virtual bool IsReadSupported() const override { return true; }
    virtual bool IsWriteSupported() const override { return false; }
    virtual bool AllowNotification() const override { return true; }
    virtual void WriteValue(std::span<const uint8_t> data) override { throw std::logic_error("Unsupported"); }
};

struct IReadOnlyCharacteristic : ICharacteristic
{
    virtual bool IsReadSupported() const override { return true; }
    virtual bool IsWriteSupported() const override { return false; }
    virtual bool AllowNotification() const override { return true; }
    virtual void WriteValue(std::span<const uint8_t> data) override { throw std::logic_error("Unsupported"); }
};

struct IService
{
    virtual UUID GetUUID() const = 0;
    virtual bool Advertise() const { return false; }

    virtual size_t           CharacteristicsCount() const   = 0;
    virtual ICharacteristic& CharacteristicAt(size_t index) = 0;

    std::unique_ptr<IBackendHandler> _handle;
};

template <typename... TChrs> struct Service : IService
{
    virtual size_t CharacteristicsCount() const override { return sizeof...(TChrs); }

    template <size_t N> ICharacteristic& _GetAt(size_t index)
    {
        if (index == N)
        {
            return std::get<N>(_chrs);
        }
        if constexpr (N > 0)
        {
            return _GetAt<N - 1>(index);
        }
        throw std::out_of_range("Cannot determine the chars");
    }

    virtual ICharacteristic& CharacteristicAt(size_t index) override
    {
        if (index >= sizeof...(TChrs))
        {
            throw std::out_of_range("Cannot determine the chars");
        }
        return _GetAt<sizeof...(TChrs) - 1>(index);
    }

    template <typename T> T& Get() { return std::get<T>(_chrs); }

    std::tuple<TChrs...> _chrs;
};

struct IApplication
{
    virtual UUID             GetUUID() const = 0;
    virtual std::string_view Name() const    = 0;

    virtual size_t    ServiceCount() const    = 0;
    virtual IService& ServiceAt(size_t index) = 0;

    void Start();

    std::unique_ptr<IBackendHandler> _handle;
};

template <typename... TChrs> using AdvService   = Service<TChrs...>;
template <typename... TChrs> using QuietService = Service<TChrs...>;

}    // namespace blegatt