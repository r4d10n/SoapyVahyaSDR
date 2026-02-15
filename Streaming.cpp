#include "SoapyVahyaSDR.hpp"
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <chrono>
#include <cstring>
#include <algorithm>

// ==== libvahya RX callbacks (called from libusb event thread) ====

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

// ==== libvahya TX callbacks (called from libusb event thread) ====

int SoapyVahyaSDR::txCallback900(int16_t *iq_data, size_t max_samples, void *ctx)
{
    auto *self = static_cast<SoapyVahyaSDR *>(ctx);
    if (!self->_txStream || !self->_txStream->active) return 0;

    RingBuffer &rb = self->_txStream->ringbufs[0];
    size_t got = rb.read(iq_data, max_samples);
    if (got == 0) {
        rb.underflow.store(true, std::memory_order_release);
    }
    return static_cast<int>(got);
}

int SoapyVahyaSDR::txCallback2400(int16_t *iq_data, size_t max_samples, void *ctx)
{
    auto *self = static_cast<SoapyVahyaSDR *>(ctx);
    if (!self->_txStream || !self->_txStream->active) return 0;

    RingBuffer &rb = self->_txStream->ringbufs[1];
    size_t got = rb.read(iq_data, max_samples);
    if (got == 0) {
        rb.underflow.store(true, std::memory_order_release);
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
    fullScale = 8192.0;  // 13-bit signed values from AT86RF215
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
    if (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32) {
        throw std::runtime_error("SoapyVahyaSDR: unsupported stream format '" + format +
                                 "', supported: CS16, CF32");
    }

    std::vector<size_t> chans = channels;
    if (chans.empty()) chans.push_back(0);

    for (auto ch : chans) {
        if (ch > 1) {
            throw std::runtime_error("SoapyVahyaSDR: invalid channel " +
                                      std::to_string(ch) + ", valid: 0 (900MHz), 1 (2.4GHz)");
        }
    }

    if (direction == SOAPY_SDR_RX && _rxStream) {
        throw std::runtime_error("SoapyVahyaSDR: RX stream already open");
    }
    if (direction == SOAPY_SDR_TX && _txStream) {
        throw std::runtime_error("SoapyVahyaSDR: TX stream already open");
    }

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

    for (auto ch : chans) {
        stream->ringbufs[ch].allocate(ringSize);
    }

    if (direction == SOAPY_SDR_RX) {
        _rxStream = stream;
    } else {
        _txStream = stream;
    }

    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyVahyaSDR: %s stream setup, format=%s, "
                   "%zu channel(s), bufflen=%zu",
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

    // Reset ring buffers before starting
    for (auto ch : vs->channels) {
        vs->ringbufs[ch].reset();
    }

    vs->active = true;

    if (vs->direction == SOAPY_SDR_RX) {
        for (auto ch : vs->channels) {
            vahya_band_t band = channelToBand(ch);

            // Configure radio for this channel
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

            // Start async USB streaming with our callback
            vahya_rx_cb cb = (ch == 0) ? rxCallback900 : rxCallback2400;
            int rc = vahya_rx_start(_dev, band, cb, this);
            if (rc != 0) {
                SoapySDR::logf(SOAPY_SDR_ERROR,
                    "SoapyVahyaSDR: rx_start failed for ch%zu (rc=%d)", ch, rc);
                vs->active = false;
                return SOAPY_SDR_STREAM_ERROR;
            }

            SoapySDR::logf(SOAPY_SDR_INFO,
                "SoapyVahyaSDR: RX ch%zu activated at %.3f MHz, SR=%.0f, BW=%.0f",
                ch, _centerFreq[ch]/1e6, _sampleRate[ch], _bandwidth[ch]);
        }
    }
    else {
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

            vahya_tx_cb cb = (ch == 0) ? txCallback900 : txCallback2400;
            int rc = vahya_tx_start(_dev, band, cb, this);
            if (rc != 0) {
                SoapySDR::logf(SOAPY_SDR_ERROR,
                    "SoapyVahyaSDR: tx_start failed for ch%zu (rc=%d)", ch, rc);
                vs->active = false;
                return SOAPY_SDR_STREAM_ERROR;
            }

            SoapySDR::logf(SOAPY_SDR_INFO,
                "SoapyVahyaSDR: TX ch%zu activated at %.3f MHz, power=%d",
                ch, _centerFreq[ch]/1e6, _txPower[ch]);
        }
    }

    return 0;
}

int SoapyVahyaSDR::deactivateStream(SoapySDR::Stream *stream, const int /*flags*/,
                                      const long long /*timeNs*/)
{
    auto *vs = reinterpret_cast<VahyaStream *>(stream);
    if (!vs || !vs->active) return 0;

    vs->active = false;

    // Wake any threads blocked in readStream/writeStream
    for (auto ch : vs->channels) {
        vs->ringbufs[ch].cv.notify_all();
    }

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

    size_t toRead = std::min(numElems, STREAM_MTU);

    // Process each requested channel, writing to the corresponding buffs[] slot
    for (size_t bi = 0; bi < vs->channels.size(); bi++) {
        size_t ch = vs->channels[bi];
        RingBuffer &rb = vs->ringbufs[ch];

        // Wait for data with timeout (only block on the first channel)
        if (bi == 0 && rb.available() < toRead) {
            std::unique_lock<std::mutex> lock(rb.mtx);
            auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::microseconds(timeoutUs);
            rb.cv.wait_until(lock, deadline, [&]() {
                return rb.available() >= toRead || !vs->active;
            });

            if (!vs->active) return SOAPY_SDR_STREAM_ERROR;
            if (rb.available() == 0) return SOAPY_SDR_TIMEOUT;

            // Adjust toRead to what's available
            toRead = std::min(toRead, rb.available());
        }

        if (!buffs[bi]) continue;

        if (vs->format == SOAPY_SDR_CS16) {
            auto *out = static_cast<int16_t *>(buffs[bi]);
            rb.read(out, toRead);
        }
        else if (vs->format == SOAPY_SDR_CF32) {
            // Read CS16 into temp buffer, convert to CF32
            std::vector<int16_t> tmp(toRead * 2);
            size_t got = rb.read(tmp.data(), toRead);

            auto *out = static_cast<float *>(buffs[bi]);
            constexpr float scale = 1.0f / 8192.0f;
            for (size_t i = 0; i < got * 2; i++) {
                out[i] = static_cast<float>(tmp[i]) * scale;
            }
        }
    }

    // Report overflow if detected
    for (auto ch : vs->channels) {
        if (vs->ringbufs[ch].overflow.exchange(false)) {
            SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyVahyaSDR: RX ch%zu ring buffer overflow", ch);
            return SOAPY_SDR_OVERFLOW;
        }
    }

    return static_cast<int>(toRead);
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

    flags = 0;
    size_t toWrite = std::min(numElems, STREAM_MTU);

    for (size_t bi = 0; bi < vs->channels.size(); bi++) {
        size_t ch = vs->channels[bi];
        RingBuffer &rb = vs->ringbufs[ch];

        // Wait for space (only block on first channel)
        if (bi == 0 && rb.space() < toWrite) {
            std::unique_lock<std::mutex> lock(rb.mtx);
            auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::microseconds(timeoutUs);
            rb.cv.wait_until(lock, deadline, [&]() {
                return rb.space() >= toWrite || !vs->active;
            });

            if (!vs->active) return SOAPY_SDR_STREAM_ERROR;
            if (rb.space() == 0) return SOAPY_SDR_TIMEOUT;

            toWrite = std::min(toWrite, rb.space());
        }

        if (!buffs[bi]) continue;

        if (vs->format == SOAPY_SDR_CS16) {
            auto *in = static_cast<const int16_t *>(buffs[bi]);
            rb.write(in, toWrite);
        }
        else if (vs->format == SOAPY_SDR_CF32) {
            auto *in = static_cast<const float *>(buffs[bi]);
            std::vector<int16_t> tmp(toWrite * 2);
            constexpr float scale = 8192.0f;
            for (size_t i = 0; i < toWrite * 2; i++) {
                float val = in[i] * scale;
                val = std::max(-8192.0f, std::min(8191.0f, val));
                tmp[i] = static_cast<int16_t>(val);
            }
            rb.write(tmp.data(), toWrite);
        }
    }

    // Report underflow if detected on any channel
    for (auto ch : vs->channels) {
        if (vs->ringbufs[ch].underflow.exchange(false)) {
            SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyVahyaSDR: TX ch%zu ring buffer underflow", ch);
        }
    }

    return static_cast<int>(toWrite);
}
