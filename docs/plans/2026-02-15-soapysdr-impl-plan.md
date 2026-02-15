# SoapyVahyaSDR Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a SoapySDR driver module that wraps libvahya to expose the Vahya Mini dual-band SDR to GQRX, GNURadio, SDR++, and all SoapySDR-compatible apps.

**Architecture:** Shared library module (`libSoapyVahyaSDR.so`) implementing `SoapySDR::Device`. Uses libvahya for USB control and async streaming. Bridges libvahya's callback-based streaming to SoapySDR's blocking `readStream()`/`writeStream()` via a lock-free ring buffer with condition variable signaling.

**Tech Stack:** C++17, SoapySDR 0.8.x, libvahya, libusb-1.0, pthreads, CMake 3.13+

**Reference files:**
- libvahya API: `/home/rfsoc/exp/Vahya/radio_gateware/host/lib/vahya.h`
- AT86RF215 registers: `/home/rfsoc/exp/Vahya/radio_gateware/host/lib/at86rf215_regs.h`
- libvahya impl: `/home/rfsoc/exp/Vahya/radio_gateware/host/lib/vahya.c`
- SoapySDR Device API: `/usr/include/SoapySDR/Device.hpp`
- SoapySDR Registry: `/usr/include/SoapySDR/Registry.hpp`
- SoapySDR module dir: `/usr/lib/x86_64-linux-gnu/SoapySDR/modules0.8/`

---

### Task 1: CMakeLists.txt and Build System

**Files:**
- Create: `CMakeLists.txt`

**Step 1: Write CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.13)
project(SoapyVahyaSDR VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SOAPY REQUIRED SoapySDR)

# libvahya from radio_gateware
set(VAHYA_LIB_DIR "${CMAKE_SOURCE_DIR}/../radio_gateware/host/lib")

add_library(SoapyVahyaSDR MODULE
    SoapyVahyaSDR.cpp
    Registration.cpp
    Streaming.cpp
    Settings.cpp
)

target_include_directories(SoapyVahyaSDR PRIVATE
    ${SOAPY_INCLUDE_DIRS}
    ${VAHYA_LIB_DIR}
)

target_link_directories(SoapyVahyaSDR PRIVATE
    ${SOAPY_LIBRARY_DIRS}
)

# Build libvahya as a static library for the module
add_library(vahya_static STATIC
    ${VAHYA_LIB_DIR}/vahya.c
)
target_include_directories(vahya_static PUBLIC ${VAHYA_LIB_DIR})
target_link_libraries(vahya_static PUBLIC usb-1.0 pthread)

target_link_libraries(SoapyVahyaSDR PRIVATE
    ${SOAPY_LIBRARIES}
    vahya_static
)

# Hide internal symbols
set_property(TARGET SoapyVahyaSDR PROPERTY CXX_VISIBILITY_PRESET hidden)

# Install to SoapySDR modules directory
set(SOAPY_MODULE_DIR "${SOAPY_LIBRARY_DIRS}/SoapySDR/modules0.8"
    CACHE PATH "SoapySDR module install directory")
install(TARGETS SoapyVahyaSDR DESTINATION ${SOAPY_MODULE_DIR})
```

**Step 2: Verify build system finds dependencies**

Run: `mkdir -p build && cd build && cmake .. 2>&1`
Expected: CMake configures successfully, finds SoapySDR and libvahya.

(Actual build will fail until source files exist — that's expected.)

**Step 3: Commit**

```bash
git add CMakeLists.txt
git commit -m "feat: add CMake build system for SoapyVahyaSDR module"
```

---

### Task 2: Device Header — SoapyVahyaSDR.hpp

**Files:**
- Create: `SoapyVahyaSDR.hpp`

**Step 1: Write the device class declaration**

```cpp
#pragma once

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>

#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <string>
#include <cstring>

extern "C" {
#include "vahya.h"
#include "at86rf215_regs.h"
}

// Ring buffer for bridging libvahya callbacks to SoapySDR blocking reads/writes
struct RingBuffer {
    std::vector<int16_t> buf;   // IQ interleaved: [I0, Q0, I1, Q1, ...]
    size_t capacity;            // in samples (each sample = 2 int16_t)
    std::atomic<size_t> head;   // write position (in samples)
    std::atomic<size_t> tail;   // read position (in samples)
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> overflow;
    std::atomic<bool> underflow;

    RingBuffer() : capacity(0), head(0), tail(0), overflow(false), underflow(false) {}

    void allocate(size_t num_samples) {
        capacity = num_samples;
        buf.resize(num_samples * 2);  // 2 int16_t per IQ sample
        head.store(0);
        tail.store(0);
        overflow.store(false);
        underflow.store(false);
    }

    size_t available() const {
        size_t h = head.load(std::memory_order_acquire);
        size_t t = tail.load(std::memory_order_acquire);
        return (h >= t) ? (h - t) : (capacity - t + h);
    }

    size_t space() const {
        return capacity - 1 - available();
    }

    // Write samples into the ring buffer (called from libvahya RX callback)
    size_t write(const int16_t *data, size_t num_samples) {
        size_t avail_space = space();
        if (num_samples > avail_space) {
            num_samples = avail_space;
            overflow.store(true, std::memory_order_release);
        }
        if (num_samples == 0) return 0;

        size_t h = head.load(std::memory_order_relaxed);
        size_t first = std::min(num_samples, capacity - h);
        std::memcpy(&buf[h * 2], data, first * 2 * sizeof(int16_t));
        if (first < num_samples) {
            std::memcpy(&buf[0], data + first * 2,
                        (num_samples - first) * 2 * sizeof(int16_t));
        }
        head.store((h + num_samples) % capacity, std::memory_order_release);
        cv.notify_one();
        return num_samples;
    }

    // Read samples from the ring buffer (called from SoapySDR readStream)
    size_t read(int16_t *data, size_t num_samples) {
        size_t avail = available();
        if (num_samples > avail) {
            num_samples = avail;
        }
        if (num_samples == 0) return 0;

        size_t t = tail.load(std::memory_order_relaxed);
        size_t first = std::min(num_samples, capacity - t);
        std::memcpy(data, &buf[t * 2], first * 2 * sizeof(int16_t));
        if (first < num_samples) {
            std::memcpy(data + first * 2, &buf[0],
                        (num_samples - first) * 2 * sizeof(int16_t));
        }
        tail.store((t + num_samples) % capacity, std::memory_order_release);
        return num_samples;
    }
};

// Stream handle
struct VahyaStream {
    int direction;              // SOAPY_SDR_RX or SOAPY_SDR_TX
    std::string format;         // "CS16" or "CF32"
    std::vector<size_t> channels;
    bool active;

    // Per-channel ring buffers
    RingBuffer ringbufs[2];     // index 0 = ch0 (900MHz), index 1 = ch1 (2.4GHz)

    // Conversion buffer for CF32
    std::vector<float> conv_buf;

    VahyaStream() : direction(-1), active(false) {}
};

class SoapyVahyaSDR : public SoapySDR::Device {
public:
    SoapyVahyaSDR(const SoapySDR::Kwargs &args);
    ~SoapyVahyaSDR() override;

    // Identification
    std::string getDriverKey() const override;
    std::string getHardwareKey() const override;
    SoapySDR::Kwargs getHardwareInfo() const override;

    // Channels
    size_t getNumChannels(const int direction) const override;
    SoapySDR::Kwargs getChannelInfo(const int direction, const size_t channel) const override;
    bool getFullDuplex(const int direction, const size_t channel) const override;

    // Antenna
    std::vector<std::string> listAntennas(const int direction, const size_t channel) const override;
    void setAntenna(const int direction, const size_t channel, const std::string &name) override;
    std::string getAntenna(const int direction, const size_t channel) const override;

    // Gain
    std::vector<std::string> listGains(const int direction, const size_t channel) const override;
    bool hasGainMode(const int direction, const size_t channel) const override;
    void setGainMode(const int direction, const size_t channel, const bool automatic) override;
    bool getGainMode(const int direction, const size_t channel) const override;
    void setGain(const int direction, const size_t channel, const std::string &name, const double value) override;
    double getGain(const int direction, const size_t channel, const std::string &name) const override;
    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const override;

    // Frequency
    void setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args) override;
    double getFrequency(const int direction, const size_t channel, const std::string &name) const override;
    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const override;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string &name) const override;

    // Sample Rate
    void setSampleRate(const int direction, const size_t channel, const double rate) override;
    double getSampleRate(const int direction, const size_t channel) const override;
    std::vector<double> listSampleRates(const int direction, const size_t channel) const override;
    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const override;

    // Bandwidth
    void setBandwidth(const int direction, const size_t channel, const double bw) override;
    double getBandwidth(const int direction, const size_t channel) const override;
    std::vector<double> listBandwidths(const int direction, const size_t channel) const override;
    SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const override;

    // Streaming
    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const override;
    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const override;
    SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const override;
    SoapySDR::Stream *setupStream(const int direction, const std::string &format,
        const std::vector<size_t> &channels, const SoapySDR::Kwargs &args) override;
    void closeStream(SoapySDR::Stream *stream) override;
    size_t getStreamMTU(SoapySDR::Stream *stream) const override;
    int activateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs, const size_t numElems) override;
    int deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs) override;
    int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems,
        int &flags, long long &timeNs, const long timeoutUs) override;
    int writeStream(SoapySDR::Stream *stream, const void *const *buffs, const size_t numElems,
        int &flags, const long long timeNs, const long timeoutUs) override;

    // Sensors
    std::vector<std::string> listSensors() const override;
    SoapySDR::ArgInfo getSensorInfo(const std::string &key) const override;
    std::string readSensor(const std::string &key) const override;
    std::vector<std::string> listSensors(const int direction, const size_t channel) const override;
    SoapySDR::ArgInfo getSensorInfo(const int direction, const size_t channel, const std::string &key) const override;
    std::string readSensor(const int direction, const size_t channel, const std::string &key) const override;

    // Register interface
    std::vector<std::string> listRegisterInterfaces() const override;
    void writeRegister(const std::string &name, const unsigned addr, const unsigned value) override;
    unsigned readRegister(const std::string &name, const unsigned addr) const override;

    // Settings
    SoapySDR::ArgInfoList getSettingInfo() const override;
    void writeSetting(const std::string &key, const std::string &value) override;
    std::string readSetting(const std::string &key) const override;

private:
    vahya_band_t channelToBand(size_t channel) const;
    uint8_t sampleRateToCode(double rate) const;
    double codeToSampleRate(uint8_t code) const;
    uint8_t bandwidthToCode(double bw) const;
    double codeToBandwidth(uint8_t code) const;

    // libvahya callbacks (static, forward to instance)
    static void rxCallback900(const int16_t *iq_data, size_t num_samples, void *ctx);
    static void rxCallback2400(const int16_t *iq_data, size_t num_samples, void *ctx);
    static int txCallback900(int16_t *iq_data, size_t max_samples, void *ctx);
    static int txCallback2400(int16_t *iq_data, size_t max_samples, void *ctx);

    vahya_device_t *_dev;
    mutable std::mutex _ctrl_mtx;   // protects SPI / control operations

    // Per-channel cached state [0]=900MHz, [1]=2.4GHz
    double _centerFreq[2];
    double _sampleRate[2];
    double _bandwidth[2];
    uint8_t _srCode[2];
    uint8_t _bwCode[2];
    double _gain[2];                // RX: AGC target (0-42 dB), TX: TXPWR (0-31)
    bool _agcMode[2];               // true = auto (AGC enabled)
    uint8_t _txPower[2];

    // Stream pointers (one RX, one TX max)
    VahyaStream *_rxStream;
    VahyaStream *_txStream;

    // Ring buffer default size
    static constexpr size_t DEFAULT_RING_SIZE = 262144;  // samples
    static constexpr size_t STREAM_MTU = 4096;           // samples per read/write
};
```

**Step 2: Commit**

```bash
git add SoapyVahyaSDR.hpp
git commit -m "feat: add SoapyVahyaSDR device class header with ring buffer"
```

---

### Task 3: Registration and Device Enumeration — Registration.cpp

**Files:**
- Create: `Registration.cpp`

**Step 1: Write the registration and factory code**

```cpp
#include "SoapyVahyaSDR.hpp"
#include <SoapySDR/Registry.hpp>
#include <libusb-1.0/libusb.h>

static std::vector<SoapySDR::Kwargs> findVahyaSDR(const SoapySDR::Kwargs &args)
{
    std::vector<SoapySDR::Kwargs> results;

    // Check if user specified a different driver
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

    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);

    return results;
}

static SoapySDR::Device *makeVahyaSDR(const SoapySDR::Kwargs &args)
{
    return new SoapyVahyaSDR(args);
}

static SoapySDR::Registry registerVahyaSDR(
    "vahya", &findVahyaSDR, &makeVahyaSDR, SOAPY_SDR_ABI_VERSION);
```

**Step 2: Commit**

```bash
git add Registration.cpp
git commit -m "feat: add SoapySDR device registration and USB enumeration"
```

---

### Task 4: Core Device Implementation — SoapyVahyaSDR.cpp

**Files:**
- Create: `SoapyVahyaSDR.cpp`

**Step 1: Write core device logic (constructor, destructor, identification, channels, antennas)**

```cpp
#include "SoapyVahyaSDR.hpp"
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <stdexcept>
#include <cmath>
#include <algorithm>

SoapyVahyaSDR::SoapyVahyaSDR(const SoapySDR::Kwargs &args)
    : _dev(nullptr), _rxStream(nullptr), _txStream(nullptr)
{
    // Initialize cached state
    for (int i = 0; i < 2; i++) {
        _centerFreq[i] = (i == 0) ? 915e6 : 2450e6;
        _sampleRate[i] = 4e6;
        _bandwidth[i] = 2e6;
        _srCode[i] = AT86_SR_4000K;
        _bwCode[i] = AT86_BW_2000K;
        _gain[i] = 30.0;   // AGC target 3 = -30 dBm
        _agcMode[i] = true;
        _txPower[i] = 15;
    }

    _dev = vahya_open();
    if (!_dev) {
        throw std::runtime_error("SoapyVahyaSDR: failed to open Vahya device "
                                 "(VID=0x1209 PID=0x0001)");
    }

    // Verify AT86RF215IQ is present
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
                                 std::to_string(partNum) + " (expected 0x35)");
    }

    SoapySDR::logf(SOAPY_SDR_INFO,
        "SoapyVahyaSDR: opened device, AT86RF215IQ part=0x%02x ver=0x%02x",
        partNum, version);

    // Set LED to green to indicate SoapySDR has control
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
    return 2;  // Ch0=900MHz, Ch1=2.4GHz
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
    // Single antenna per channel, nothing to switch
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
    // Find closest available bandwidth
    struct { double hz; uint8_t code; } bws[] = {
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
    for (auto &b : bws) {
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
```

**Step 2: Commit**

```bash
git add SoapyVahyaSDR.cpp
git commit -m "feat: implement core device logic, channels, antennas, helpers"
```

---

### Task 5: Settings Implementation — Settings.cpp

**Files:**
- Create: `Settings.cpp`

**Step 1: Write frequency, gain, sample rate, bandwidth, sensors, registers, and settings**

```cpp
#include "SoapyVahyaSDR.hpp"
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <cmath>
#include <algorithm>

// ==== Gain ====

std::vector<std::string> SoapyVahyaSDR::listGains(const int direction, const size_t /*channel*/) const
{
    if (direction == SOAPY_SDR_RX) return {"AGC"};
    return {"TXPWR"};
}

bool SoapyVahyaSDR::hasGainMode(const int direction, const size_t /*channel*/) const
{
    return (direction == SOAPY_SDR_RX);
}

void SoapyVahyaSDR::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    if (direction != SOAPY_SDR_RX) return;
    if (channel > 1) return;

    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _agcMode[channel] = automatic;

    vahya_band_t band = channelToBand(channel);
    uint16_t agcc_reg = (band == VAHYA_BAND_900) ? AT86_RF09_AGCC : AT86_RF24_AGCC;

    uint8_t val = AT86_AGCC_AGCI | AT86_AGCC_AVGS_32;
    if (automatic) val |= AT86_AGCC_EN;
    // If manual, AGC_EN=0 freezes at current gain

    vahya_spi_write(_dev, agcc_reg, val);

    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyVahyaSDR: ch%zu AGC mode=%s",
                   channel, automatic ? "auto" : "manual");
}

bool SoapyVahyaSDR::getGainMode(const int /*direction*/, const size_t channel) const
{
    return (channel <= 1) ? _agcMode[channel] : true;
}

void SoapyVahyaSDR::setGain(const int direction, const size_t channel,
                             const std::string &name, const double value)
{
    if (channel > 1) return;
    std::lock_guard<std::mutex> lock(_ctrl_mtx);

    vahya_band_t band = channelToBand(channel);

    if (direction == SOAPY_SDR_RX && name == "AGC") {
        // Map 0-42 dB gain to AGC target 7-0 (inverted: higher gain = lower target)
        // AGC target: 0=-21dBm, 7=-42dBm, 3dB steps
        int target = 7 - static_cast<int>(std::round(value / 3.0));
        target = std::max(0, std::min(7, target));

        uint16_t agcs_reg = (band == VAHYA_BAND_900) ? AT86_RF09_AGCS : AT86_RF24_AGCS;
        vahya_spi_write(_dev, agcs_reg, AT86_AGCS_TGT(target));
        _gain[channel] = (7 - target) * 3.0;

        SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyVahyaSDR: ch%zu RX gain=%.0f dB (AGC target=%d)",
                       channel, _gain[channel], target);
    }
    else if (direction == SOAPY_SDR_TX && name == "TXPWR") {
        uint8_t pwr = static_cast<uint8_t>(std::max(0.0, std::min(31.0, value)));
        _txPower[channel] = pwr;
        _gain[channel] = pwr;

        uint16_t pac_reg = (band == VAHYA_BAND_900) ? AT86_RF09_PAC : AT86_RF24_PAC;
        vahya_spi_write(_dev, pac_reg,
                        AT86_PAC_PACUR(AT86_PACUR_0MA) | (pwr & 0x1F));

        SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyVahyaSDR: ch%zu TX power=%d",
                       channel, pwr);
    }
}

double SoapyVahyaSDR::getGain(const int /*direction*/, const size_t channel,
                               const std::string &/*name*/) const
{
    return (channel <= 1) ? _gain[channel] : 0.0;
}

SoapySDR::Range SoapyVahyaSDR::getGainRange(const int direction, const size_t /*channel*/,
                                              const std::string &/*name*/) const
{
    if (direction == SOAPY_SDR_RX)
        return SoapySDR::Range(0.0, 42.0, 3.0);  // AGC target 7..0 * 3dB
    return SoapySDR::Range(0.0, 31.0, 1.0);       // TX power 0-31
}

// ==== Frequency ====

void SoapyVahyaSDR::setFrequency(const int direction, const size_t channel,
                                   const std::string &name, const double frequency,
                                   const SoapySDR::Kwargs &/*args*/)
{
    if (channel > 1) return;
    if (name != "RF" && !name.empty()) return;

    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _centerFreq[channel] = frequency;

    vahya_band_t band = channelToBand(channel);

    // Re-init radio with new frequency
    if (direction == SOAPY_SDR_RX) {
        vahya_rx_config_t cfg;
        cfg.band = band;
        cfg.freq_hz = static_cast<uint32_t>(frequency);
        cfg.bw = _bwCode[channel];
        cfg.sample_rate = _srCode[channel];
        vahya_radio_init_rx(_dev, &cfg);
    } else {
        vahya_tx_config_t cfg;
        cfg.band = band;
        cfg.freq_hz = static_cast<uint32_t>(frequency);
        cfg.bw = _bwCode[channel];
        cfg.sample_rate = _srCode[channel];
        cfg.tx_power = _txPower[channel];
        vahya_radio_init_tx(_dev, &cfg);
    }

    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyVahyaSDR: ch%zu %s freq=%.3f MHz",
                   channel, (direction == SOAPY_SDR_RX) ? "RX" : "TX",
                   frequency / 1e6);
}

double SoapyVahyaSDR::getFrequency(const int /*direction*/, const size_t channel,
                                     const std::string &/*name*/) const
{
    return (channel <= 1) ? _centerFreq[channel] : 0.0;
}

std::vector<std::string> SoapyVahyaSDR::listFrequencies(const int /*direction*/,
                                                          const size_t /*channel*/) const
{
    return {"RF"};
}

SoapySDR::RangeList SoapyVahyaSDR::getFrequencyRange(const int /*direction*/,
                                                        const size_t channel,
                                                        const std::string &/*name*/) const
{
    if (channel == 0) {
        return {SoapySDR::Range(377e6, 1020e6)};
    }
    return {SoapySDR::Range(2400e6, 2550e6)};
}

// ==== Sample Rate ====

void SoapyVahyaSDR::setSampleRate(const int direction, const size_t channel, const double rate)
{
    if (channel > 1) return;

    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _srCode[channel] = sampleRateToCode(rate);
    _sampleRate[channel] = codeToSampleRate(_srCode[channel]);

    // Apply to hardware by re-initializing
    vahya_band_t band = channelToBand(channel);

    if (direction == SOAPY_SDR_RX) {
        vahya_rx_config_t cfg;
        cfg.band = band;
        cfg.freq_hz = static_cast<uint32_t>(_centerFreq[channel]);
        cfg.bw = _bwCode[channel];
        cfg.sample_rate = _srCode[channel];
        vahya_radio_init_rx(_dev, &cfg);
    } else {
        vahya_tx_config_t cfg;
        cfg.band = band;
        cfg.freq_hz = static_cast<uint32_t>(_centerFreq[channel]);
        cfg.bw = _bwCode[channel];
        cfg.sample_rate = _srCode[channel];
        cfg.tx_power = _txPower[channel];
        vahya_radio_init_tx(_dev, &cfg);
    }

    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyVahyaSDR: ch%zu sample rate=%.0f Hz (code=0x%02x)",
                   channel, _sampleRate[channel], _srCode[channel]);
}

double SoapyVahyaSDR::getSampleRate(const int /*direction*/, const size_t channel) const
{
    return (channel <= 1) ? _sampleRate[channel] : 4e6;
}

std::vector<double> SoapyVahyaSDR::listSampleRates(const int /*direction*/,
                                                      const size_t /*channel*/) const
{
    return {800e3, 1e6, 4e6/3.0, 2e6, 4e6};
}

SoapySDR::RangeList SoapyVahyaSDR::getSampleRateRange(const int /*direction*/,
                                                         const size_t /*channel*/) const
{
    return {
        SoapySDR::Range(800e3, 800e3),
        SoapySDR::Range(1e6, 1e6),
        SoapySDR::Range(4e6/3.0, 4e6/3.0),
        SoapySDR::Range(2e6, 2e6),
        SoapySDR::Range(4e6, 4e6),
    };
}

// ==== Bandwidth ====

void SoapyVahyaSDR::setBandwidth(const int direction, const size_t channel, const double bw)
{
    if (channel > 1) return;

    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _bwCode[channel] = bandwidthToCode(bw);
    _bandwidth[channel] = codeToBandwidth(_bwCode[channel]);

    // Apply via re-init
    vahya_band_t band = channelToBand(channel);

    if (direction == SOAPY_SDR_RX) {
        vahya_rx_config_t cfg;
        cfg.band = band;
        cfg.freq_hz = static_cast<uint32_t>(_centerFreq[channel]);
        cfg.bw = _bwCode[channel];
        cfg.sample_rate = _srCode[channel];
        vahya_radio_init_rx(_dev, &cfg);
    } else {
        vahya_tx_config_t cfg;
        cfg.band = band;
        cfg.freq_hz = static_cast<uint32_t>(_centerFreq[channel]);
        cfg.bw = _bwCode[channel];
        cfg.sample_rate = _srCode[channel];
        cfg.tx_power = _txPower[channel];
        vahya_radio_init_tx(_dev, &cfg);
    }

    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyVahyaSDR: ch%zu bandwidth=%.0f Hz",
                   channel, _bandwidth[channel]);
}

double SoapyVahyaSDR::getBandwidth(const int /*direction*/, const size_t channel) const
{
    return (channel <= 1) ? _bandwidth[channel] : 2e6;
}

std::vector<double> SoapyVahyaSDR::listBandwidths(const int /*direction*/,
                                                     const size_t /*channel*/) const
{
    return {160e3, 200e3, 250e3, 400e3, 500e3, 1e6, 1.25e6, 1.6e6, 2e6};
}

SoapySDR::RangeList SoapyVahyaSDR::getBandwidthRange(const int /*direction*/,
                                                        const size_t /*channel*/) const
{
    return {SoapySDR::Range(160e3, 2e6)};
}

// ==== Sensors ====

std::vector<std::string> SoapyVahyaSDR::listSensors() const
{
    return {"fw_version"};
}

SoapySDR::ArgInfo SoapyVahyaSDR::getSensorInfo(const std::string &key) const
{
    SoapySDR::ArgInfo info;
    if (key == "fw_version") {
        info.key = "fw_version";
        info.name = "Firmware Version";
        info.description = "Vahya gateware version";
        info.type = SoapySDR::ArgInfo::STRING;
    }
    return info;
}

std::string SoapyVahyaSDR::readSensor(const std::string &key) const
{
    if (key == "fw_version" && _dev) {
        vahya_status_t status;
        if (vahya_get_status(const_cast<vahya_device_t*>(_dev), &status) == 0) {
            return std::to_string(status.fw_version_major) + "." +
                   std::to_string(status.fw_version_minor);
        }
    }
    return "";
}

std::vector<std::string> SoapyVahyaSDR::listSensors(const int direction,
                                                       const size_t /*channel*/) const
{
    if (direction == SOAPY_SDR_RX) return {"rssi", "rx_overflow", "pll_locked"};
    return {"tx_underflow", "pll_locked"};
}

SoapySDR::ArgInfo SoapyVahyaSDR::getSensorInfo(const int /*direction*/,
                                                  const size_t /*channel*/,
                                                  const std::string &key) const
{
    SoapySDR::ArgInfo info;
    info.key = key;

    if (key == "rssi") {
        info.name = "RSSI";
        info.description = "Received signal strength in dBm";
        info.type = SoapySDR::ArgInfo::FLOAT;
        info.units = "dBm";
    } else if (key == "rx_overflow") {
        info.name = "RX Overflow";
        info.description = "FPGA RX FIFO overflow detected";
        info.type = SoapySDR::ArgInfo::BOOL;
    } else if (key == "tx_underflow") {
        info.name = "TX Underflow";
        info.description = "FPGA TX FIFO underflow detected";
        info.type = SoapySDR::ArgInfo::BOOL;
    } else if (key == "pll_locked") {
        info.name = "PLL Locked";
        info.description = "AT86RF215 PLL lock status";
        info.type = SoapySDR::ArgInfo::BOOL;
    }
    return info;
}

std::string SoapyVahyaSDR::readSensor(const int direction, const size_t channel,
                                        const std::string &key) const
{
    if (!_dev || channel > 1) return "";

    vahya_band_t band = (channel == 0) ? VAHYA_BAND_900 : VAHYA_BAND_2400;

    if (key == "rssi" && direction == SOAPY_SDR_RX) {
        int8_t rssi = 0;
        if (vahya_read_rssi(const_cast<vahya_device_t*>(_dev), band, &rssi) == 0) {
            return std::to_string(static_cast<int>(rssi));
        }
    }
    else if (key == "rx_overflow") {
        vahya_status_t status;
        if (vahya_get_status(const_cast<vahya_device_t*>(_dev), &status) == 0) {
            bool overflow = (channel == 0) ?
                (status.rx_fifo_status & 0x01) : (status.rx_fifo_status & 0x02);
            return overflow ? "true" : "false";
        }
    }
    else if (key == "tx_underflow") {
        vahya_status_t status;
        if (vahya_get_status(const_cast<vahya_device_t*>(_dev), &status) == 0) {
            bool underflow = (channel == 0) ?
                (status.tx_fifo_status & 0x01) : (status.tx_fifo_status & 0x02);
            return underflow ? "true" : "false";
        }
    }
    else if (key == "pll_locked") {
        uint16_t pll_reg = (band == VAHYA_BAND_900) ? AT86_RF09_PLL : AT86_RF24_PLL;
        uint8_t val = 0;
        if (vahya_spi_read(const_cast<vahya_device_t*>(_dev), pll_reg, &val) == 0) {
            return (val & AT86_PLL_LOCKED) ? "true" : "false";
        }
    }

    return "";
}

// ==== Register Interface ====

std::vector<std::string> SoapyVahyaSDR::listRegisterInterfaces() const
{
    return {"AT86RF215"};
}

void SoapyVahyaSDR::writeRegister(const std::string &name, const unsigned addr,
                                    const unsigned value)
{
    if (name != "AT86RF215") return;
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    vahya_spi_write(_dev, static_cast<uint16_t>(addr), static_cast<uint8_t>(value));
}

unsigned SoapyVahyaSDR::readRegister(const std::string &name, const unsigned addr) const
{
    if (name != "AT86RF215") return 0;
    uint8_t val = 0;
    vahya_spi_read(const_cast<vahya_device_t*>(_dev), static_cast<uint16_t>(addr), &val);
    return val;
}

// ==== Settings ====

SoapySDR::ArgInfoList SoapyVahyaSDR::getSettingInfo() const
{
    SoapySDR::ArgInfoList settings;

    {
        SoapySDR::ArgInfo info;
        info.key = "ifs_shift";
        info.name = "IF Shift";
        info.description = "Enable IF frequency shift x1.25 for wider bandwidth";
        info.type = SoapySDR::ArgInfo::BOOL;
        info.value = "true";
        settings.push_back(info);
    }
    {
        SoapySDR::ArgInfo info;
        info.key = "fine_freq";
        info.name = "Fine Frequency";
        info.description = "Enable fine frequency resolution for RF09 (sub-GHz)";
        info.type = SoapySDR::ArgInfo::BOOL;
        info.value = "true";
        settings.push_back(info);
    }
    {
        SoapySDR::ArgInfo info;
        info.key = "pa_ramp";
        info.name = "PA Ramp Time";
        info.description = "Power amplifier ramp time: 0=4us, 1=8us, 2=16us, 3=32us";
        info.type = SoapySDR::ArgInfo::INT;
        info.value = "2";
        info.range = SoapySDR::Range(0, 3, 1);
        settings.push_back(info);
    }
    {
        SoapySDR::ArgInfo info;
        info.key = "led_color";
        info.name = "LED Color";
        info.description = "RGB LED control as 'R,G,B' (0 or 1 each)";
        info.type = SoapySDR::ArgInfo::STRING;
        info.value = "0,1,0";
        settings.push_back(info);
    }

    return settings;
}

void SoapyVahyaSDR::writeSetting(const std::string &key, const std::string &value)
{
    std::lock_guard<std::mutex> lock(_ctrl_mtx);

    if (key == "ifs_shift") {
        vahya_features_t feat = *vahya_get_features(_dev);
        feat.ifs_shift = (value == "true" || value == "1") ? 1 : 0;
        vahya_set_features(_dev, &feat);
    }
    else if (key == "fine_freq") {
        vahya_features_t feat = *vahya_get_features(_dev);
        feat.fine_freq_rf09 = (value == "true" || value == "1") ? 1 : 0;
        vahya_set_features(_dev, &feat);
    }
    else if (key == "pa_ramp") {
        vahya_features_t feat = *vahya_get_features(_dev);
        feat.pa_ramp_time = static_cast<uint8_t>(std::stoi(value)) & 0x03;
        vahya_set_features(_dev, &feat);
    }
    else if (key == "led_color") {
        // Parse "R,G,B"
        uint8_t r = 0, g = 0, b = 0;
        if (sscanf(value.c_str(), "%hhu,%hhu,%hhu", &r, &g, &b) == 3) {
            vahya_set_led(_dev, r & 1, g & 1, b & 1);
        }
    }
}

std::string SoapyVahyaSDR::readSetting(const std::string &key) const
{
    if (!_dev) return "";

    const vahya_features_t *feat = vahya_get_features(const_cast<vahya_device_t*>(_dev));
    if (!feat) return "";

    if (key == "ifs_shift") return feat->ifs_shift ? "true" : "false";
    if (key == "fine_freq") return feat->fine_freq_rf09 ? "true" : "false";
    if (key == "pa_ramp") return std::to_string(feat->pa_ramp_time);
    if (key == "led_color") return "0,0,0";

    return "";
}
```

**Step 2: Commit**

```bash
git add Settings.cpp
git commit -m "feat: implement frequency, gain, sample rate, bandwidth, sensors, registers, settings"
```

---

### Task 6: Streaming Implementation — Streaming.cpp

**Files:**
- Create: `Streaming.cpp`

**Step 1: Write streaming code with ring buffer bridge**

```cpp
#include "SoapyVahyaSDR.hpp"
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <chrono>
#include <cstring>
#include <algorithm>

// ==== libvahya RX callbacks ====

void SoapyVahyaSDR::rxCallback900(const int16_t *iq_data, size_t num_samples, void *ctx)
{
    auto *self = static_cast<SoapyVahyaSDR *>(ctx);
    if (self->_rxStream && self->_rxStream->active) {
        self->_rxStream->ringbufs[0].write(iq_data, num_samples);
    }
}

void SoapyVahyaSDR::rxCallback2400(const int16_t *iq_data, size_t num_samples, void *ctx)
{
    auto *self = static_cast<SoapyVahyaSDR *>(ctx);
    if (self->_rxStream && self->_rxStream->active) {
        self->_rxStream->ringbufs[1].write(iq_data, num_samples);
    }
}

// ==== libvahya TX callbacks ====

int SoapyVahyaSDR::txCallback900(int16_t *iq_data, size_t max_samples, void *ctx)
{
    auto *self = static_cast<SoapyVahyaSDR *>(ctx);
    if (!self->_txStream || !self->_txStream->active) return 0;

    size_t got = self->_txStream->ringbufs[0].read(iq_data, max_samples);
    if (got == 0) {
        self->_txStream->ringbufs[0].underflow.store(true, std::memory_order_release);
    }
    return static_cast<int>(got);
}

int SoapyVahyaSDR::txCallback2400(int16_t *iq_data, size_t max_samples, void *ctx)
{
    auto *self = static_cast<SoapyVahyaSDR *>(ctx);
    if (!self->_txStream || !self->_txStream->active) return 0;

    size_t got = self->_txStream->ringbufs[1].read(iq_data, max_samples);
    if (got == 0) {
        self->_txStream->ringbufs[1].underflow.store(true, std::memory_order_release);
    }
    return static_cast<int>(got);
}

// ==== Stream format info ====

std::vector<std::string> SoapyVahyaSDR::getStreamFormats(const int /*direction*/,
                                                           const size_t /*channel*/) const
{
    return {SOAPY_SDR_CS16, SOAPY_SDR_CF32};
}

std::string SoapyVahyaSDR::getNativeStreamFormat(const int /*direction*/,
                                                   const size_t /*channel*/,
                                                   double &fullScale) const
{
    fullScale = 8192.0;  // 13-bit signed
    return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList SoapyVahyaSDR::getStreamArgsInfo(const int /*direction*/,
                                                          const size_t /*channel*/) const
{
    SoapySDR::ArgInfoList args;

    SoapySDR::ArgInfo buffLen;
    buffLen.key = "bufflen";
    buffLen.name = "Buffer Length";
    buffLen.description = "Ring buffer size in samples";
    buffLen.type = SoapySDR::ArgInfo::INT;
    buffLen.value = std::to_string(DEFAULT_RING_SIZE);
    args.push_back(buffLen);

    return args;
}

// ==== Stream setup/close ====

SoapySDR::Stream *SoapyVahyaSDR::setupStream(const int direction,
                                                const std::string &format,
                                                const std::vector<size_t> &channels,
                                                const SoapySDR::Kwargs &args)
{
    // Validate format
    if (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32) {
        throw std::runtime_error("SoapyVahyaSDR: unsupported format '" + format +
                                 "', use CS16 or CF32");
    }

    // Default channels: just channel 0
    std::vector<size_t> chans = channels;
    if (chans.empty()) chans.push_back(0);

    for (auto ch : chans) {
        if (ch > 1) throw std::runtime_error("SoapyVahyaSDR: invalid channel " +
                                              std::to_string(ch));
    }

    // Check for duplicate stream
    if (direction == SOAPY_SDR_RX && _rxStream) {
        throw std::runtime_error("SoapyVahyaSDR: RX stream already open");
    }
    if (direction == SOAPY_SDR_TX && _txStream) {
        throw std::runtime_error("SoapyVahyaSDR: TX stream already open");
    }

    // Parse ring buffer size
    size_t ringSize = DEFAULT_RING_SIZE;
    if (args.count("bufflen")) {
        ringSize = std::stoul(args.at("bufflen"));
        if (ringSize < 1024) ringSize = 1024;
    }

    auto *stream = new VahyaStream();
    stream->direction = direction;
    stream->format = format;
    stream->channels = chans;
    stream->active = false;

    // Allocate ring buffers for requested channels
    for (auto ch : chans) {
        stream->ringbufs[ch].allocate(ringSize);
    }

    // Allocate conversion buffer for CF32
    if (format == SOAPY_SDR_CF32) {
        stream->conv_buf.resize(STREAM_MTU * 2);  // I + Q floats
    }

    if (direction == SOAPY_SDR_RX) {
        _rxStream = stream;
    } else {
        _txStream = stream;
    }

    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyVahyaSDR: %s stream setup, format=%s, "
                   "channels=%zu, bufflen=%zu",
                   (direction == SOAPY_SDR_RX) ? "RX" : "TX",
                   format.c_str(), chans.size(), ringSize);

    return reinterpret_cast<SoapySDR::Stream *>(stream);
}

void SoapyVahyaSDR::closeStream(SoapySDR::Stream *stream)
{
    auto *vs = reinterpret_cast<VahyaStream *>(stream);
    if (!vs) return;

    if (vs->active) {
        deactivateStream(stream, 0, 0);
    }

    if (vs == _rxStream) _rxStream = nullptr;
    if (vs == _txStream) _txStream = nullptr;

    delete vs;
}

size_t SoapyVahyaSDR::getStreamMTU(SoapySDR::Stream */*stream*/) const
{
    return STREAM_MTU;
}

// ==== Stream activate/deactivate ====

int SoapyVahyaSDR::activateStream(SoapySDR::Stream *stream, const int /*flags*/,
                                    const long long /*timeNs*/, const size_t /*numElems*/)
{
    auto *vs = reinterpret_cast<VahyaStream *>(stream);
    if (!vs || vs->active) return 0;

    vs->active = true;

    if (vs->direction == SOAPY_SDR_RX) {
        // Initialize radio for each channel, then start streaming
        for (auto ch : vs->channels) {
            vahya_band_t band = channelToBand(ch);

            vahya_rx_config_t cfg;
            cfg.band = band;
            cfg.freq_hz = static_cast<uint32_t>(_centerFreq[ch]);
            cfg.bw = _bwCode[ch];
            cfg.sample_rate = _srCode[ch];

            {
                std::lock_guard<std::mutex> lock(_ctrl_mtx);
                int rc = vahya_radio_init_rx(_dev, &cfg);
                if (rc != 0) {
                    SoapySDR::logf(SOAPY_SDR_ERROR,
                        "SoapyVahyaSDR: radio_init_rx failed for ch%zu (rc=%d)", ch, rc);
                    vs->active = false;
                    return SOAPY_SDR_STREAM_ERROR;
                }
            }

            auto cb = (ch == 0) ? rxCallback900 : rxCallback2400;
            int rc = vahya_rx_start(_dev, band, cb, this);
            if (rc != 0) {
                SoapySDR::logf(SOAPY_SDR_ERROR,
                    "SoapyVahyaSDR: rx_start failed for ch%zu (rc=%d)", ch, rc);
                vs->active = false;
                return SOAPY_SDR_STREAM_ERROR;
            }
        }
    }
    else {
        // TX: init radio, streaming starts on first writeStream
        for (auto ch : vs->channels) {
            vahya_band_t band = channelToBand(ch);

            vahya_tx_config_t cfg;
            cfg.band = band;
            cfg.freq_hz = static_cast<uint32_t>(_centerFreq[ch]);
            cfg.bw = _bwCode[ch];
            cfg.sample_rate = _srCode[ch];
            cfg.tx_power = _txPower[ch];

            {
                std::lock_guard<std::mutex> lock(_ctrl_mtx);
                int rc = vahya_radio_init_tx(_dev, &cfg);
                if (rc != 0) {
                    SoapySDR::logf(SOAPY_SDR_ERROR,
                        "SoapyVahyaSDR: radio_init_tx failed for ch%zu (rc=%d)", ch, rc);
                    vs->active = false;
                    return SOAPY_SDR_STREAM_ERROR;
                }
            }

            auto cb = (ch == 0) ? txCallback900 : txCallback2400;
            int rc = vahya_tx_start(_dev, band, cb, this);
            if (rc != 0) {
                SoapySDR::logf(SOAPY_SDR_ERROR,
                    "SoapyVahyaSDR: tx_start failed for ch%zu (rc=%d)", ch, rc);
                vs->active = false;
                return SOAPY_SDR_STREAM_ERROR;
            }
        }
    }

    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyVahyaSDR: %s stream activated",
                   (vs->direction == SOAPY_SDR_RX) ? "RX" : "TX");
    return 0;
}

int SoapyVahyaSDR::deactivateStream(SoapySDR::Stream *stream, const int /*flags*/,
                                      const long long /*timeNs*/)
{
    auto *vs = reinterpret_cast<VahyaStream *>(stream);
    if (!vs || !vs->active) return 0;

    vs->active = false;

    for (auto ch : vs->channels) {
        vahya_band_t band = channelToBand(ch);
        if (vs->direction == SOAPY_SDR_RX) {
            vahya_rx_stop(_dev, band);
        } else {
            vahya_tx_stop(_dev, band);
        }
    }

    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyVahyaSDR: %s stream deactivated",
                   (vs->direction == SOAPY_SDR_RX) ? "RX" : "TX");
    return 0;
}

// ==== readStream ====

int SoapyVahyaSDR::readStream(SoapySDR::Stream *stream, void *const *buffs,
                                const size_t numElems, int &flags,
                                long long &timeNs, const long timeoutUs)
{
    auto *vs = reinterpret_cast<VahyaStream *>(stream);
    if (!vs || !vs->active || vs->direction != SOAPY_SDR_RX) {
        return SOAPY_SDR_STREAM_ERROR;
    }

    flags = 0;
    timeNs = 0;

    // Use the first requested channel
    size_t ch = vs->channels[0];
    RingBuffer &rb = vs->ringbufs[ch];

    // Wait for data with timeout
    size_t toRead = std::min(numElems, STREAM_MTU);

    if (rb.available() < toRead) {
        std::unique_lock<std::mutex> lock(rb.mtx);
        auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::microseconds(timeoutUs);
        rb.cv.wait_until(lock, deadline, [&]() {
            return rb.available() >= toRead || !vs->active;
        });

        if (!vs->active) return SOAPY_SDR_STREAM_ERROR;
        if (rb.available() == 0) return SOAPY_SDR_TIMEOUT;
    }

    // Read available samples (may be less than requested)
    toRead = std::min(toRead, rb.available());

    if (vs->format == SOAPY_SDR_CS16) {
        // Direct copy — native format
        int16_t *out = static_cast<int16_t *>(buffs[0]);
        size_t got = rb.read(out, toRead);

        // Handle multi-channel: copy second channel if requested
        if (vs->channels.size() > 1 && buffs[1]) {
            size_t ch2 = vs->channels[1];
            size_t toRead2 = std::min(got, vs->ringbufs[ch2].available());
            int16_t *out2 = static_cast<int16_t *>(buffs[1]);
            vs->ringbufs[ch2].read(out2, toRead2);
        }

        // Check overflow
        if (rb.overflow.exchange(false)) {
            flags |= SOAPY_SDR_END_ABRUPT;
            return SOAPY_SDR_OVERFLOW;
        }

        return static_cast<int>(got);
    }
    else if (vs->format == SOAPY_SDR_CF32) {
        // Read CS16, convert to CF32
        // Use a local buffer to avoid allocation
        std::vector<int16_t> tmp(toRead * 2);
        size_t got = rb.read(tmp.data(), toRead);

        float *out = static_cast<float *>(buffs[0]);
        constexpr float scale = 1.0f / 8192.0f;  // 2^13
        for (size_t i = 0; i < got * 2; i++) {
            out[i] = static_cast<float>(tmp[i]) * scale;
        }

        // Handle multi-channel
        if (vs->channels.size() > 1 && buffs[1]) {
            size_t ch2 = vs->channels[1];
            size_t toRead2 = std::min(got, vs->ringbufs[ch2].available());
            std::vector<int16_t> tmp2(toRead2 * 2);
            vs->ringbufs[ch2].read(tmp2.data(), toRead2);

            float *out2 = static_cast<float *>(buffs[1]);
            for (size_t i = 0; i < toRead2 * 2; i++) {
                out2[i] = static_cast<float>(tmp2[i]) * scale;
            }
        }

        if (rb.overflow.exchange(false)) {
            flags |= SOAPY_SDR_END_ABRUPT;
            return SOAPY_SDR_OVERFLOW;
        }

        return static_cast<int>(got);
    }

    return SOAPY_SDR_STREAM_ERROR;
}

// ==== writeStream ====

int SoapyVahyaSDR::writeStream(SoapySDR::Stream *stream, const void *const *buffs,
                                 const size_t numElems, int &flags,
                                 const long long /*timeNs*/, const long timeoutUs)
{
    auto *vs = reinterpret_cast<VahyaStream *>(stream);
    if (!vs || !vs->active || vs->direction != SOAPY_SDR_TX) {
        return SOAPY_SDR_STREAM_ERROR;
    }

    size_t ch = vs->channels[0];
    RingBuffer &rb = vs->ringbufs[ch];

    size_t toWrite = std::min(numElems, STREAM_MTU);

    // Wait for space
    if (rb.space() < toWrite) {
        std::unique_lock<std::mutex> lock(rb.mtx);
        auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::microseconds(timeoutUs);
        rb.cv.wait_until(lock, deadline, [&]() {
            return rb.space() >= toWrite || !vs->active;
        });

        if (!vs->active) return SOAPY_SDR_STREAM_ERROR;
        if (rb.space() == 0) return SOAPY_SDR_TIMEOUT;
    }

    toWrite = std::min(toWrite, rb.space());

    if (vs->format == SOAPY_SDR_CS16) {
        const int16_t *in = static_cast<const int16_t *>(buffs[0]);
        size_t wrote = rb.write(in, toWrite);

        // Multi-channel
        if (vs->channels.size() > 1 && buffs[1]) {
            size_t ch2 = vs->channels[1];
            const int16_t *in2 = static_cast<const int16_t *>(buffs[1]);
            vs->ringbufs[ch2].write(in2, toWrite);
        }

        return static_cast<int>(wrote);
    }
    else if (vs->format == SOAPY_SDR_CF32) {
        // Convert CF32 to CS16
        const float *in = static_cast<const float *>(buffs[0]);
        std::vector<int16_t> tmp(toWrite * 2);
        constexpr float scale = 8192.0f;
        for (size_t i = 0; i < toWrite * 2; i++) {
            float val = in[i] * scale;
            val = std::max(-8192.0f, std::min(8191.0f, val));
            tmp[i] = static_cast<int16_t>(val);
        }
        size_t wrote = rb.write(tmp.data(), toWrite);

        // Multi-channel
        if (vs->channels.size() > 1 && buffs[1]) {
            size_t ch2 = vs->channels[1];
            const float *in2 = static_cast<const float *>(buffs[1]);
            std::vector<int16_t> tmp2(toWrite * 2);
            for (size_t i = 0; i < toWrite * 2; i++) {
                float val = in2[i] * scale;
                val = std::max(-8192.0f, std::min(8191.0f, val));
                tmp2[i] = static_cast<int16_t>(val);
            }
            vs->ringbufs[ch2].write(tmp2.data(), toWrite);
        }

        return static_cast<int>(wrote);
    }

    flags = 0;
    return SOAPY_SDR_STREAM_ERROR;
}
```

**Step 2: Commit**

```bash
git add Streaming.cpp
git commit -m "feat: implement streaming with ring buffer bridge (RX + TX, CS16 + CF32)"
```

---

### Task 7: Build, Test, and Install

**Step 1: Build the module**

Run: `cd build && cmake .. && make -j$(nproc) 2>&1`
Expected: Compiles without errors, produces `libSoapyVahyaSDR.so`.

**Step 2: Fix any compilation errors**

Address compiler warnings and errors as they appear.

**Step 3: Install the module**

Run: `sudo cp build/libSoapyVahyaSDR.so /usr/lib/x86_64-linux-gnu/SoapySDR/modules0.8/`
Expected: Module installed.

**Step 4: Verify module loads**

Run: `SoapySDRUtil --info 2>&1 | grep -i vahya`
Expected: Shows "vahya" in the list of available factories.

**Step 5: Probe device (if hardware connected)**

Run: `SoapySDRUtil --probe="driver=vahya" 2>&1`
Expected: Shows device info, channels, frequency ranges, sample rates, gains.

**Step 6: Commit any fixes**

```bash
git add -A
git commit -m "fix: resolve build issues and verify SoapySDR module loading"
```

---

### Task 8: Final verification and push

**Step 1: Run SoapySDRUtil --probe to verify full API**

Run: `SoapySDRUtil --probe="driver=vahya" 2>&1`
Expected: Full probe output showing all capabilities.

**Step 2: Test with a real SDR app (if hardware connected)**

Run GQRX or `SoapySDRUtil --rate` to verify streaming works.

**Step 3: Final commit and push**

```bash
git add -A
git status
git log --oneline
git push origin main
```
