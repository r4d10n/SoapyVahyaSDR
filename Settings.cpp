#include "SoapyVahyaSDR.hpp"
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <cmath>
#include <algorithm>
#include <cstdio>

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
    if (direction != SOAPY_SDR_RX || channel > 1) return;

    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _agcMode[channel] = automatic;

    vahya_band_t band = channelToBand(channel);
    uint16_t agcc_reg = (band == VAHYA_BAND_900) ? AT86_RF09_AGCC : AT86_RF24_AGCC;

    uint8_t val = AT86_AGCC_AGCI | AT86_AGCC_AVGS_32;
    if (automatic) val |= AT86_AGCC_EN;

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
        // Map 0-42 dB gain to AGC target 7-0
        // AGC target: 0=-21dBm, 7=-42dBm (3dB steps)
        // Higher gain value = lower target number = more amplification
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

        SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyVahyaSDR: ch%zu TX power=%d", channel, pwr);
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
        return SoapySDR::Range(0.0, 21.0, 3.0);  // AGC target 7..0 * 3dB
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

    if (direction == SOAPY_SDR_RX) {
        vahya_rx_config_t cfg;
        cfg.band = band;
        cfg.freq_hz = static_cast<uint32_t>(frequency);
        cfg.bw = _bwCode[channel];
        cfg.sample_rate = _srCode[channel];

        int rc = vahya_radio_init_rx(_dev, &cfg);
        if (rc != 0) {
            SoapySDR::logf(SOAPY_SDR_ERROR,
                "SoapyVahyaSDR: setFrequency RX ch%zu failed (rc=%d)", channel, rc);
        }
    } else {
        vahya_tx_config_t cfg;
        cfg.band = band;
        cfg.freq_hz = static_cast<uint32_t>(frequency);
        cfg.bw = _bwCode[channel];
        cfg.sample_rate = _srCode[channel];
        cfg.tx_power = _txPower[channel];

        int rc = vahya_radio_init_tx(_dev, &cfg);
        if (rc != 0) {
            SoapySDR::logf(SOAPY_SDR_ERROR,
                "SoapyVahyaSDR: setFrequency TX ch%zu failed (rc=%d)", channel, rc);
        }
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
            bool uf = (channel == 0) ?
                (status.tx_fifo_status & 0x01) : (status.tx_fifo_status & 0x02);
            return uf ? "true" : "false";
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
        uint8_t r = 0, g = 0, b = 0;
        if (std::sscanf(value.c_str(), "%hhu,%hhu,%hhu", &r, &g, &b) == 3) {
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
