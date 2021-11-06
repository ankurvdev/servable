#include "BleApplication.h"

#include <iostream>

using namespace blegatt;
#define TODO() throw std::logic_error("Not Impl");

struct LNFeature : IReadOnlyCharacteristic
{
    virtual UUID   GetUUID() const override { return "00002A6A-0000-1000-8000-00805F9B34FB"; }
    virtual size_t ReadValue(std::span<uint8_t> buffer) override { TODO(); }
};

struct LocationAndSpeed : IReadNotifyCharacteristic
{
    virtual UUID   GetUUID() const override { return "00002A67-0000-1000-8000-00805F9B34FB"; }
    virtual size_t ReadValue(std::span<uint8_t> buffer) override { TODO(); }
};

struct PositionQuality : IReadNotifyCharacteristic
{
    virtual UUID   GetUUID() const override { return "00002A69-0000-1000-8000-00805F9B34FB"; }
    virtual size_t ReadValue(std::span<uint8_t> buffer) override { TODO(); }
};

struct LocationAndNavigation : IService
{
    virtual UUID             GetUUID() const override { return "00001819-0000-1000-8000-00805F9B34FB"; }
    virtual size_t           CharacteristicsCount() const override { return 3; }
    virtual ICharacteristic& CharacteristicAt(size_t index) override
    {
        ICharacteristic* chars[] = {&_lnfeature, &_locAndSpeed, &_posQuality};
        return *chars[index];
    }

    virtual bool Advertise() const override { return true; }

    LNFeature        _lnfeature;
    LocationAndSpeed _locAndSpeed;
    PositionQuality  _posQuality;
};

struct AvidServer : IApplication
{
    virtual UUID             GetUUID() const override { return "00001819-0000-1000-8000-00805f9b34fb"; }
    virtual std::string_view Name() const override { return "Avid"; }

    virtual size_t    ServiceCount() const override { return 1; }
    virtual IService& ServiceAt(size_t index) override
    {
        IService* svcs[] = {&_locNavSvc};
        return *svcs[index];
    }

    private:
    LocationAndNavigation _locNavSvc;
};

int main(int argc, const char* argv[])
try
{
    AvidServer server;
    server.Start();
    return 0;
} catch (std::exception const& ex)
{
    // std::cerr << ex.what() << std::endl;
    return -1;
}