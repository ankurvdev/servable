#pragma once
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
    UUID(std::string_view str) { bt_string_to_uuid(this, str.data()); }
    UUID(char const* const str) { bt_string_to_uuid(this, str); }
};

struct ICharacteristic
{
    virtual UUID GetUUID() const = 0;

    virtual bool IsReadSupported() const   = 0;
    virtual bool IsWriteSupported() const  = 0;
    virtual bool AllowNotification() const = 0;

    virtual size_t ReadValue(std::span<uint8_t> buffer)      = 0;
    virtual void   WriteValue(std::span<const uint8_t> data) = 0;
    void           NotifyUpdated() { throw std::logic_error("Not Implemented"); }

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

    std::tuple<TChrs...> _chrs;
};

template <typename... TChrs> using AdvService   = Service<TChrs...>;
template <typename... TChrs> using QuietService = Service<TChrs...>;

struct IApplication
{
    virtual UUID             GetUUID() const = 0;
    virtual std::string_view Name() const    = 0;

    virtual size_t    ServiceCount() const    = 0;
    virtual IService& ServiceAt(size_t index) = 0;

    void Start();

    std::unique_ptr<IBackendHandler> _handle;
};

}    // namespace blegatt