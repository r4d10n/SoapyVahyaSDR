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

// Ring buffer for bridging libvahya callbacks to SoapySDR blocking reads/writes.
// Single-producer single-consumer with condition variable signaling.
struct RingBuffer {
    std::vector<int16_t> buf;   // IQ interleaved: [I0, Q0, I1, Q1, ...]
    size_t capacity;            // in samples (each sample = 2 int16_t values)
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
        head.store(0, std::memory_order_relaxed);
        tail.store(0, std::memory_order_relaxed);
        overflow.store(false, std::memory_order_relaxed);
        underflow.store(false, std::memory_order_relaxed);
    }

    void reset() {
        head.store(0, std::memory_order_relaxed);
        tail.store(0, std::memory_order_relaxed);
        overflow.store(false, std::memory_order_relaxed);
        underflow.store(false, std::memory_order_relaxed);
    }

    size_t available() const {
        size_t h = head.load(std::memory_order_acquire);
        size_t t = tail.load(std::memory_order_acquire);
        if (h >= t) return h - t;
        return capacity - t + h;
    }

    size_t space() const {
        return capacity - 1 - available();
    }

    // Write samples into the ring buffer (called from libvahya RX callback thread)
    size_t write(const int16_t *data, size_t num_samples) {
        size_t avail_space = space();
        if (num_samples > avail_space) {
            num_samples = avail_space;
            overflow.store(true, std::memory_order_release);
        }
        if (num_samples == 0) return 0;

        size_t h = head.load(std::memory_order_relaxed);
        size_t first = (num_samples <= capacity - h) ? num_samples : (capacity - h);
        std::memcpy(&buf[h * 2], data, first * 2 * sizeof(int16_t));
        if (first < num_samples) {
            std::memcpy(&buf[0], data + first * 2,
                        (num_samples - first) * 2 * sizeof(int16_t));
        }
        head.store((h + num_samples) % capacity, std::memory_order_release);
        cv.notify_one();
        return num_samples;
    }

    // Read samples from the ring buffer (called from SoapySDR readStream thread)
    size_t read(int16_t *data, size_t num_samples) {
        size_t avail = available();
        if (num_samples > avail) num_samples = avail;
        if (num_samples == 0) return 0;

        size_t t = tail.load(std::memory_order_relaxed);
        size_t first = (num_samples <= capacity - t) ? num_samples : (capacity - t);
        std::memcpy(data, &buf[t * 2], first * 2 * sizeof(int16_t));
        if (first < num_samples) {
            std::memcpy(data + first * 2, &buf[0],
                        (num_samples - first) * 2 * sizeof(int16_t));
        }
        tail.store((t + num_samples) % capacity, std::memory_order_release);
        return num_samples;
    }
};

// Stream handle exposed to SoapySDR as opaque Stream*
struct VahyaStream {
    int direction;              // SOAPY_SDR_RX or SOAPY_SDR_TX
    std::string format;         // "CS16" or "CF32"
    std::vector<size_t> channels;
    bool active;

    // Per-channel ring buffers (index 0 = ch0/900MHz, index 1 = ch1/2.4GHz)
    RingBuffer ringbufs[2];

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

    // libvahya callbacks (static, forward to instance via ctx pointer)
    static void rxCallback900(const int16_t *iq_data, size_t num_samples, void *ctx);
    static void rxCallback2400(const int16_t *iq_data, size_t num_samples, void *ctx);
    static int txCallback900(int16_t *iq_data, size_t max_samples, void *ctx);
    static int txCallback2400(int16_t *iq_data, size_t max_samples, void *ctx);

    vahya_device_t *_dev;
    mutable std::mutex _ctrl_mtx;   // protects SPI / control operations

    // Per-channel cached state [0]=900MHz (ch0), [1]=2.4GHz (ch1)
    double _centerFreq[2];
    double _sampleRate[2];
    double _bandwidth[2];
    uint8_t _srCode[2];
    uint8_t _bwCode[2];
    double _gain[2];
    bool _agcMode[2];
    uint8_t _txPower[2];

    // Stream pointers (one RX, one TX max)
    VahyaStream *_rxStream;
    VahyaStream *_txStream;

    static constexpr size_t DEFAULT_RING_SIZE = 262144;  // samples
    static constexpr size_t STREAM_MTU = 4096;           // samples per read/write
};
