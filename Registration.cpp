#include "SoapyVahyaSDR.hpp"
#include <SoapySDR/Registry.hpp>
#include <libusb-1.0/libusb.h>

static std::vector<SoapySDR::Kwargs> findVahyaSDR(const SoapySDR::Kwargs &args)
{
    std::vector<SoapySDR::Kwargs> results;

    // If user specified a different driver, skip enumeration
    if (args.count("driver") && args.at("driver") != "vahya") {
        return results;
    }

    libusb_context *ctx = nullptr;
    if (libusb_init(&ctx) != 0) {
        return results;
    }

    libusb_device **devs = nullptr;
    ssize_t cnt = libusb_get_device_list(ctx, &devs);

    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(devs[i], &desc) != 0)
            continue;

        if (desc.idVendor == VAHYA_VID && desc.idProduct == VAHYA_PID) {
            SoapySDR::Kwargs devInfo;
            devInfo["driver"] = "vahya";
            devInfo["label"] = "Vahya Mini v1.0b [AT86RF215IQ]";

            uint8_t bus = libusb_get_bus_number(devs[i]);
            uint8_t addr = libusb_get_device_address(devs[i]);
            devInfo["serial"] = std::to_string(bus) + ":" + std::to_string(addr);

            results.push_back(devInfo);
        }
    }

    if (devs) libusb_free_device_list(devs, 1);
    libusb_exit(ctx);

    return results;
}

static SoapySDR::Device *makeVahyaSDR(const SoapySDR::Kwargs &args)
{
    return new SoapyVahyaSDR(args);
}

static SoapySDR::Registry registerVahyaSDR(
    "vahya", &findVahyaSDR, &makeVahyaSDR, SOAPY_SDR_ABI_VERSION);
