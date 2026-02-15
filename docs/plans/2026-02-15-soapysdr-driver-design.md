# SoapyVahyaSDR Driver Design

**Date:** 2026-02-15
**Status:** Approved

## Overview

SoapySDR driver module for the Vahya Mini v1.0b SDR — a Lattice ECP5 FPGA +
AT86RF215IQ dual-band (900 MHz + 2.4 GHz) transceiver over USB 2.0 High-Speed.

The driver wraps the existing `libvahya` C library, which handles USB vendor
requests, SPI register access, async bulk streaming, and all hardware
workarounds (TRXOFF errata retry, PLL lock verification, TX IQ calibration,
bit-slip frame sync).

## Architecture

```
SDR App (GQRX / GNURadio / SDR++ / CubicSDR)
        |  SoapySDR API (CF32 / CS16)
  libSoapyVahyaSDR.so
        |  libvahya API (CS16 native)
      libvahya.so
        |  libusb (USB 2.0 bulk + vendor requests)
    Vahya Mini FPGA (ECP5)
        |  SPI + LVDS
     AT86RF215IQ
```

## Channel Model

Two channels per direction, mapping directly to the AT86RF215IQ dual-band
subsystems:

| Direction | Channel 0           | Channel 1              |
|-----------|---------------------|------------------------|
| RX        | 900 MHz (EP1 0x81)  | 2.4 GHz (EP2 0x82)    |
| TX        | 900 MHz (EP3 0x03)  | 2.4 GHz (EP4 0x04)    |

Both directions support full duplex.  Each channel is independently
configurable for frequency, sample rate, bandwidth, and gain.

## Frequency Ranges

| Channel      | Range               | Resolution                          |
|--------------|---------------------|-------------------------------------|
| Ch0 (RF09)   | 377 MHz – 1020 MHz  | ~99 Hz (fine) / 25 kHz (IEEE)      |
| Ch1 (RF24)   | 2400 MHz – 2550 MHz | 25 kHz (IEEE mode)                  |

Fine resolution modes are auto-selected for RF09 when the `fine_freq` feature
flag is enabled (default).

## Sample Rates and Bandwidths

Five discrete sample rates supported by the AT86RF215 digital front-end:

| Sample Rate (Hz) | RXDFE code | Default BW (Hz) | RXBWC code |
|-------------------|------------|------------------|------------|
| 4,000,000         | 0x01       | 2,000,000        | 0x0B       |
| 2,000,000         | 0x02       | 1,600,000        | 0x0A       |
| 1,333,333         | 0x03       | 1,250,000        | 0x09       |
| 1,000,000         | 0x04       | 1,000,000        | 0x08       |
| 800,000           | 0x05       | 500,000          | 0x05       |

Bandwidth can be set independently.  Available bandwidths:
160k, 200k, 250k, 400k, 500k, 1000k, 1250k, 1600k, 2000k Hz.

## Gain Model

### RX Gain

Single gain element `"AGC"`:
- Range: 0 dB to 42 dB (mapped from AGC target levels)
- AGC target field (0-7) maps to -21 dBm to -42 dBm in 3 dB steps
- Gain mode: auto (AGC enabled, default) or manual (AGC frozen)
- In manual mode, GCW (gain control word, 0-23) provides direct control

### TX Gain

Single gain element `"TXPWR"`:
- Range: 0 to 31 (PA power level, PAC register bits [4:0])
- 0 = minimum power, 31 = maximum power

## Stream Format Support

- **Native format:** `CS16` (complex signed 16-bit, 4 bytes/sample)
  - Hardware produces 13-bit signed values sign-extended to 16-bit
- **Converted format:** `CF32` (complex float32, 8 bytes/sample)
  - Normalized to [-1.0, +1.0] by dividing by 8192.0 (2^13)

## Streaming Architecture

libvahya provides callback-based async streaming (16-deep USB transfer queue,
16 KB per transfer).  SoapySDR requires blocking `readStream()`/`writeStream()`.

### Bridge Design

**RX path:** libvahya RX callback pushes CS16 samples into a lock-free ring
buffer.  `readStream()` reads from the ring buffer, optionally converting
CS16 → CF32.  Returns `SOAPY_SDR_OVERFLOW` when the ring buffer overflows.

**TX path:** `writeStream()` pushes samples into a ring buffer (converting
CF32 → CS16 if needed).  libvahya TX callback pulls from the ring buffer.
Returns `SOAPY_SDR_UNDERFLOW` when the ring buffer is empty.

**Ring buffer size:** Configurable via stream arg `bufflen`, default 262144
samples (1 MB per channel at CS16).  Provides ~65 ms buffering at 4 MSps.

**Timeout:** `readStream()`/`writeStream()` block with configurable timeout
(default 500 ms).  Uses condition variable signaling from the callback thread.

## Sensors

| Sensor          | Scope       | Type   | Description                      |
|-----------------|-------------|--------|----------------------------------|
| `rssi`          | per-channel | float  | RSSI in dBm (-127 to +4)        |
| `fw_version`    | global      | string | Firmware version "major.minor"   |
| `rx_overflow`   | per-channel | bool   | FPGA RX FIFO overflow flag       |
| `tx_underflow`  | per-channel | bool   | FPGA TX FIFO underflow flag      |
| `pll_locked`    | per-channel | bool   | PLL lock status                  |

## Register Interface

Direct AT86RF215 register access via SoapySDR register API:
- Interface name: `"AT86RF215"`
- Address range: 0x0000 – 0x1FFF (13-bit)
- Data width: 8-bit

## Settings

| Setting         | Type   | Default | Description                        |
|-----------------|--------|---------|------------------------------------|
| `ifs_shift`     | bool   | true    | IF shift x1.25 for wider BW       |
| `fine_freq`     | bool   | true    | Fine frequency resolution (RF09)   |
| `pa_ramp`       | int    | 2       | PA ramp time (0=4us..3=32us)       |
| `agc_averaging` | int    | 0       | AGC averaging (0=8..3=64 samples)  |
| `led_color`     | string | "0,0,0" | RGB LED control "R,G,B" (0 or 1)  |

## Antenna Names

Each channel reports a single antenna:
- Channel 0: `"Sub-GHz"` (900 MHz band)
- Channel 1: `"2.4GHz"` (2.4 GHz band)

## File Structure

```
SoapyVahyaSDR/
├── CMakeLists.txt              # Build system, finds libvahya + SoapySDR
├── SoapyVahyaSDR.hpp           # Device class declaration
├── Registration.cpp            # find/make factory + SoapySDR module registration
├── SoapyVahyaSDR.cpp           # Core: ctor/dtor, identification, channels, antennas
├── Streaming.cpp               # Ring buffer, stream setup/activate/read/write
├── Settings.cpp                # Frequency, gain, sample rate, BW, sensors, registers
└── docs/
    └── plans/
        └── 2026-02-15-soapysdr-driver-design.md
```

## Device Enumeration

`findVahyaSDR()` uses libusb to scan for VID=0x1209, PID=0x0001.
Returns device args with:
- `driver=vahya`
- `label=Vahya Mini v1.0b`
- `serial=<usb_bus>:<usb_addr>`

## Error Handling

- All libvahya calls checked for return codes
- USB disconnection detected, streams gracefully stopped
- PLL lock failure reported via sensor + SoapySDR log
- FIFO overflow/underflow reported via stream status flags
- Thread-safe: mutex protects libvahya control calls (SPI, config)
  while streaming callbacks run on libusb event thread

## Dependencies

- SoapySDR >= 0.8.0
- libvahya (from radio_gateware/host/lib/)
- libusb-1.0 (transitive via libvahya)
- pthreads
