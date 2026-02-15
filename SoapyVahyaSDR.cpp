#include "SoapyVahyaSDR.hpp"
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <stdexcept>
#include <cmath>
#include <algorithm>

SoapyVahyaSDR::SoapyVahyaSDR(const SoapySDR::Kwargs &/*args*/)
    : _dev(nullptr), _rxStream(nullptr), _txStream(nullptr)
{
    for (int i = 0; i < 2; i++) {
        _centerFreq[i] = (i == 0) ? 915e6 : 2450e6;
        _sampleRate[i] = 4e6;
        _bandwidth[i] = 2e6;
        _srCode[i] = AT86_SR_4000K;
        _bwCode[i] = AT86_BW_2000K;
        _gain[i] = 12.0;   // AGC target 3 = -30 dBm → 12 dB gain
        _agcMode[i] = true;
        _txPower[i] = 15;
    }

    _dev = vahya_open();
    if (!_dev) {
        throw std::runtime_error("SoapyVahyaSDR: failed to open Vahya device "
                                 "(VID=0x1209 PID=0x0001)");
    }

    // Verify AT86RF215IQ is present via SPI
    uint8_t partNum = 0, version = 0;
    if (vahya_spi_read(_dev, AT86_RF_PN, &partNum) != 0 ||
        vahya_spi_read(_dev, AT86_RF_VN, &version) != 0) {
        vahya_close(_dev);
        _dev = nullptr;
        throw std::runtime_error("SoapyVahyaSDR: SPI communication failed");
    }

    if (partNum != 0x35) {
        vahya_close(_dev);
        _dev = nullptr;
        throw std::runtime_error("SoapyVahyaSDR: unexpected part number 0x" +
                                 std::to_string(partNum) + " (expected 0x35 for IQ variant)");
    }

    SoapySDR::logf(SOAPY_SDR_INFO,
        "SoapyVahyaSDR: opened device, AT86RF215IQ part=0x%02x ver=0x%02x",
        partNum, version);

    // Green LED = SoapySDR has control
    vahya_set_led(_dev, 0, 1, 0);
}

SoapyVahyaSDR::~SoapyVahyaSDR()
{
    if (_rxStream) {
        deactivateStream(reinterpret_cast<SoapySDR::Stream *>(_rxStream), 0, 0);
        closeStream(reinterpret_cast<SoapySDR::Stream *>(_rxStream));
    }
    if (_txStream) {
        deactivateStream(reinterpret_cast<SoapySDR::Stream *>(_txStream), 0, 0);
        closeStream(reinterpret_cast<SoapySDR::Stream *>(_txStream));
    }

    if (_dev) {
        vahya_set_led(_dev, 0, 0, 0);
        vahya_close(_dev);
    }
}

// ---- Identification ----

std::string SoapyVahyaSDR::getDriverKey() const { return "vahya"; }
std::string SoapyVahyaSDR::getHardwareKey() const { return "AT86RF215IQ"; }

SoapySDR::Kwargs SoapyVahyaSDR::getHardwareInfo() const
{
    SoapySDR::Kwargs info;
    info["frontend"] = "AT86RF215IQ dual-band transceiver";
    info["fpga"] = "Lattice ECP5 LFE5U-25F";
    info["transport"] = "USB 2.0 High-Speed";
    info["board"] = "Vahya Mini v1.0b";

    if (_dev) {
        vahya_status_t status;
        if (vahya_get_status(const_cast<vahya_device_t*>(_dev), &status) == 0) {
            info["fw_version"] = std::to_string(status.fw_version_major) + "." +
                                 std::to_string(status.fw_version_minor);
        }
    }
    return info;
}

// ---- Channels ----

size_t SoapyVahyaSDR::getNumChannels(const int /*direction*/) const
{
    return 2;  // Ch0=RF09(900MHz), Ch1=RF24(2.4GHz)
}

SoapySDR::Kwargs SoapyVahyaSDR::getChannelInfo(const int /*direction*/, const size_t channel) const
{
    SoapySDR::Kwargs info;
    if (channel == 0) {
        info["name"] = "RF09";
        info["band"] = "Sub-GHz (377-1020 MHz)";
    } else {
        info["name"] = "RF24";
        info["band"] = "2.4 GHz (2400-2550 MHz)";
    }
    return info;
}

bool SoapyVahyaSDR::getFullDuplex(const int /*direction*/, const size_t /*channel*/) const
{
    return true;
}

// ---- Antenna ----

std::vector<std::string> SoapyVahyaSDR::listAntennas(const int /*direction*/, const size_t channel) const
{
    if (channel == 0) return {"Sub-GHz"};
    return {"2.4GHz"};
}

void SoapyVahyaSDR::setAntenna(const int /*direction*/, const size_t /*channel*/, const std::string &/*name*/)
{
    // Single antenna per channel — nothing to switch
}

std::string SoapyVahyaSDR::getAntenna(const int /*direction*/, const size_t channel) const
{
    return (channel == 0) ? "Sub-GHz" : "2.4GHz";
}

// ---- Helpers ----

vahya_band_t SoapyVahyaSDR::channelToBand(size_t channel) const
{
    return (channel == 0) ? VAHYA_BAND_900 : VAHYA_BAND_2400;
}

uint8_t SoapyVahyaSDR::sampleRateToCode(double rate) const
{
    // Find nearest supported sample rate
    if (rate >= 3.5e6) return AT86_SR_4000K;
    if (rate >= 1.65e6) return AT86_SR_2000K;
    if (rate >= 1.15e6) return AT86_SR_1333K;
    if (rate >= 0.9e6)  return AT86_SR_1000K;
    return AT86_SR_800K;
}

double SoapyVahyaSDR::codeToSampleRate(uint8_t code) const
{
    switch (code) {
        case AT86_SR_4000K: return 4e6;
        case AT86_SR_2000K: return 2e6;
        case AT86_SR_1333K: return 4e6 / 3.0;
        case AT86_SR_1000K: return 1e6;
        case AT86_SR_800K:  return 0.8e6;
        default: return 4e6;
    }
}

uint8_t SoapyVahyaSDR::bandwidthToCode(double bw) const
{
    struct BwEntry { double hz; uint8_t code; };
    static const BwEntry bws[] = {
        {160e3,  AT86_BW_160K},
        {200e3,  AT86_BW_200K},
        {250e3,  AT86_BW_250K},
        {400e3,  AT86_BW_400K},
        {500e3,  AT86_BW_500K},
        {1000e3, AT86_BW_1000K},
        {1250e3, AT86_BW_1250K},
        {1600e3, AT86_BW_1600K},
        {2000e3, AT86_BW_2000K},
    };

    uint8_t best = AT86_BW_2000K;
    double bestDiff = 1e12;
    for (const auto &b : bws) {
        double diff = std::abs(b.hz - bw);
        if (diff < bestDiff) {
            bestDiff = diff;
            best = b.code;
        }
    }
    return best;
}

double SoapyVahyaSDR::codeToBandwidth(uint8_t code) const
{
    switch (code & 0x0F) {
        case AT86_BW_160K:  return 160e3;
        case AT86_BW_200K:  return 200e3;
        case AT86_BW_250K:  return 250e3;
        case AT86_BW_400K:  return 400e3;
        case AT86_BW_500K:  return 500e3;
        case AT86_BW_1000K: return 1e6;
        case AT86_BW_1250K: return 1.25e6;
        case AT86_BW_1600K: return 1.6e6;
        case AT86_BW_2000K: return 2e6;
        default: return 2e6;
    }
}
