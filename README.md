# ESP32 FM Transmitter

## Overview

This project transforms an ESP32 microcontroller into a basic FM radio transmitter by leveraging its internal hardware components. It generates a carrier frequency in the FM band and modulates it with audio data to broadcast signals that can be received by standard FM radios. The implementation focuses on using the Audio Phase-Locked Loop (APLL) for precise frequency control and the Inter-IC Sound (I2S) peripheral to output the modulated signal. Audio modulation is handled through a polar coordinate-based approach, allowing for efficient amplitude and phase adjustments.

The transmitter operates at low power, making it suitable for short-range demonstrations or experiments. It includes support for various modulation modes, filters, and automatic gain control (AGC) to process input signals. Note that this is an educational and experimental tool; users should comply with local regulations regarding radio transmissions to avoid interference.

## Features

- Generates FM carrier frequencies between approximately 76 MHz and 125 MHz.
- Supports frequency modulation with configurable deviation (e.g., up to 75 kHz for wide FM).
- Includes embedded audio playback for testing, with looping capability.
- Configurable modulation types, including narrow FM, wide FM, AM, SSB, and test signals.
- Audio processing pipeline with high-pass, low-pass, and passband filters.
- Automatic gain control for consistent signal levels.
- Outputs the RF signal directly on a GPIO pin (default: GPIO0).
- Compatible only with original ESP32 chips (not S2, S3, or C3 variants due to hardware limitations).

## Requirements

### Hardware
- ESP32 development board (e.g., ESP32-WROOM or ESP32-WROVER module). Must be the original ESP32 silicon revision with APLL support.
- A short wire or antenna connected to GPIO0 for signal transmission (keep it short to limit range and comply with regulations).
- Optional: An FM radio receiver for testing reception.

### Software
- ESP-IDF v5.5.1 or compatible (tested with this version).
- Compiler and build tools for ESP32 (e.g., via ESP-IDF setup).
- No external libraries beyond ESP-IDF are required.

### Limitations
- Transmission power is very low (microwatts), resulting in a range of only a few meters.
- Only supports mono, 8-bit PCM audio at fixed sample rates (e.g., 8 kHz in the example).
- Not suitable for production broadcasting; intended for learning and prototyping.

## Installation

1. Clone the repository:
   ```
   git clone https://github.com/hiperiondev/esp32_fm_radio.git
   cd esp32_fm_radio
   ```

2. Set up ESP-IDF:
   - Follow the official ESP-IDF getting started guide: [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).
   - Ensure your environment is configured for the ESP32 target.

3. Build the project:
   ```
   idf.py build
   ```

4. Flash to the ESP32:
   ```
   idf.py -p PORT flash monitor
   ```
   Replace `PORT` with your serial port (e.g., `/dev/ttyUSB0`).

5. Tune an FM radio to the configured carrier frequency (default around 100 MHz) and place it near the ESP32 to hear the transmitted audio.

## How It Works

This section provides a detailed, step-by-step explanation of the system's operation, from frequency generation to audio modulation and signal output. The design relies on the ESP32's clocking and peripheral features to create a modulated RF signal without external RF hardware.

### 1. Carrier Frequency Generation Using APLL
The core of the transmitter is the ESP32's Audio PLL (APLL), a specialized clock generator that produces high-frequency signals based on the crystal oscillator (typically 40 MHz). The APLL is tuned to output a frequency in the FM band (e.g., 100 MHz) by adjusting internal parameters:

- **APLL Configuration Parameters**:
  - **Output Divider (o_div)**: Controls the final division of the internal voltage-controlled oscillator (VCO) frequency. Values range from 0 to 31, ensuring the VCO stays between 350 MHz and 500 MHz for stable locking.
  - **Fractional Multiplier Components (sdm2, sdm1, sdm0)**: These form a 16-bit fractional part added to an integer multiplier (starting from 4). The effective multiplier is `4 + sdm2 + (sdm1 / 256) + (sdm0 / 65536)`.
  - The output frequency is derived from the crystal frequency divided and multiplied according to these values.

- **Calculation Process**:
  - The system calculates the smallest o_div that keeps the VCO in the valid range.
  - It then computes the integer (sdm2) and fractional (sdm1:sdm0) parts to match the target carrier (e.g., 100 MHz) as closely as possible.
  - A deviation value is also precomputed, representing how many fractional steps correspond to the maximum frequency swing (e.g., ±75 kHz). This ensures symmetric modulation without overflowing the fractional range.
  - If the base fractional value is too close to boundaries (0 or 65535), it's adjusted slightly to allow full deviation.

- **Initialization**:
  - The APLL is enabled and programmed with these coefficients.
  - The produced frequency is logged, along with any minor error (typically under 100 Hz).

This setup creates a stable, unmodulated carrier wave.

### 2. I2S Peripheral Setup for Clock Output
The I2S peripheral is configured to use the APLL as its clock source and generate a master clock (MCLK) at the carrier frequency:

- **I2S Configuration**:
  - Operates in transmit (TX) master mode.
  - Clock source set to APLL.
  - Sample rate matches the audio (e.g., 8 kHz), but no actual data is transmitted—only the clock is used.
  - MCLK is routed to GPIO0 via pin multiplexing (CLK_OUT1 signal).

- **Output Routing**:
  - GPIO0 is set to output mode.
  - The MCLK signal (carrier wave) is continuously emitted as a square wave on this pin.
  - A short antenna wire on GPIO0 radiates the signal weakly.

This turns GPIO0 into the RF output port.

### 3. Audio Processing and Modulation
Audio is modulated onto the carrier using a polar modulation approach, converting amplitude variations into phase and amplitude changes:

- **Audio Source**:
  - Embedded as an 8-bit unsigned PCM array (e.g., a looped song clip at 8 kHz).
  - Samples are read sequentially and converted to signed values (-128 to 127).

- **Processing Pipeline**:
  - **Filters**: High-pass (to remove DC), low-pass (to limit bandwidth, e.g., 3-3.4 kHz), and passband shaping.
  - **AGC**: Dynamically adjusts gain to normalize levels, detecting high/low/silent audio and updating every 25 ms.
  - **Limiter**: Soft compression to prevent clipping.
  - **Hilbert Transform**: Generates in-phase (I) and quadrature (Q) components for phase calculation.
  - **CORDIC Algorithm**: Computes amplitude and phase from I/Q.
  - **Mode-Specific Modulation**:
    - For FM: Phase difference drives frequency shifts; amplitude is fixed.
    - Deviation scaled per mode (e.g., 2.5 kHz for narrow FM, 75 kHz for wide).
  - Special test modes generate tones or patterns for debugging.

- **Timer-Driven Modulation**:
  - A high-resolution timer interrupts at the audio sample rate (e.g., every 125 μs).
  - In the callback, the next sample is processed through the pipeline.
  - The computed phase difference is scaled to fractional APLL steps.
  - The APLL's fractional registers are updated dynamically, shifting the carrier frequency proportionally to the audio.

This real-time update modulates the output signal.

### 4. Signal Transmission and Reception
- The modulated square wave on GPIO0 includes harmonics, but the fundamental frequency carries the FM signal.
- A nearby FM receiver decodes the frequency variations as audio.
- Range is limited due to low power; extend cautiously with better antennas.

### 5. Status and Debugging
- Logs show APLL config, produced frequency, and errors.
- Flags track AGC state, audio levels, and overflows.

## API Documentation

### From `polar_mod.h`

#### Enumerations
- `polar_status_e`: Flags for modulator state (e.g., `PTT_ACTIVE`, `AUDIO_SILENCE`).
- `modulation_mode_t`: Modulation types (e.g., `MOD_FM` for standard FM, `MOD_FMW` for wide FM).
- `special_modulation_t`: Test signals (e.g., `SPECIAL_MODULATION_2_TONE_SIG`).
- `filter_pre_lp_t`, `filter_pre_hp_t`, `filter_pre_pb_t`, `filter_post_lp_t`: Filter options (e.g., `FILTER_LP_3000_4pol` for 4-pole 3 kHz low-pass).
- `agc_type_t`: AGC modes (e.g., `AGC_NORMAL`).

#### Structures
- `polar_mod_ctx_t`: Internal state (gain, delays, counters).
- `modulation_t`: Configuration (mode, filters, AGC, status).

#### Functions
- `void polar_mod_init(polar_mod_ctx_t *ctx)`: Resets context to defaults (gain=1000, zeros delays).
- `int modulation_am_pm(polar_mod_ctx_t *ctx, modulation_t modulation, int data, int *ampl_out, int *phase_diff_out)`: Processes one sample through filters/AGC/limiter/Hilbert/CORDIC. Outputs amplitude and phase diff. Returns 0 on success, -1 on error.

### From `esp32_tx.h`

#### Structures
- `apll_cfg_t`: APLL params (o_div, sdm2, base_frac16, dev_frac16, is_rev0).
- `tx_cfg_t`: Transmitter settings (carrier_hz, max_dev_hz, wav_sr_hz, modulation_gain).
- `wav_t`: Audio buffer (audio pointer, length).
- `tx_ctx_t`: Full context (tx_cfg, apll_cfg, polar_mod_ctx, modulation, wav).

#### Functions
- `void fm_i2s_init(tx_ctx_t tx_ctx)`: Sets up I2S TX with APLL clock at audio sample rate.
- `bool fm_apll_init(tx_ctx_t *tx_ctx)`: Computes and programs APLL for carrier/deviation. Returns true on success.
- `void fm_route_to_pin(void)`: Maps MCLK to GPIO0 as output.
- `void fm_start_audio(tx_ctx_t *tx_ctx)`: Starts timer for audio modulation updates.

## Example of Use

### Basic Main Function (`main.c`)
```c
#include "esp32_tx.h"
#include "polar_mod.h"

// Example embedded audio (replace with your PCM array)
extern const unsigned char audio_data[];
extern const unsigned int audio_len;

void app_main(void) {
    tx_ctx_t ctx = {0};

    // Configure transmitter
    ctx.tx_cfg.carrier_hz = 100000000;  // 100 MHz
    ctx.tx_cfg.max_dev_hz = 75000;      // ±75 kHz deviation
    ctx.tx_cfg.wav_sr_hz = 8000;        // 8 kHz sample rate
    ctx.tx_cfg.modulation_gain = 1;     // Default gain

    // Set audio
    ctx.wav.audio = audio_data;
    ctx.wav.audio_len = audio_len;

    // Set modulation (FM wide)
    ctx.modulation.modulation_mode = MOD_FMW;
    ctx.modulation.filter_pre_hp = FILTER_HP_300_2pol;
    ctx.modulation.filter_pre_lp = FILTER_LP_3400_4pol;
    ctx.modulation.filter_pre_pb = FILTER_PB_NONE;
    ctx.modulation.filter_post_lp = FILTER_POST_LP_3400_4pol;
    ctx.modulation.agc_type = AGC_NORMAL;
    ctx.modulation.special_modulation = SPECIAL_MODULATION_NORMAL;
    ctx.modulation.polar_status = 0;

    // Initialize modulator
    polar_mod_init(&ctx.polar_mod_ctx);

    // Setup hardware
    fm_apll_init(&ctx);
    fm_i2s_init(ctx);
    fm_route_to_pin();
    fm_start_audio(&ctx);

    // Run indefinitely
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
```

### Steps
1. Include headers and define audio array.
2. Set up `tx_ctx_t` with desired frequency, deviation, and modulation.
3. Initialize and start components.
4. Build/flash as described in Installation.
5. Tune radio to 100 MHz to hear audio.

## Troubleshooting
- No signal: Check GPIO0 connection and radio proximity.
- Distorted audio: Verify sample rate matches audio.
- APLL failure: Ensure frequency in range; check logs.

## License
MIT License. See LICENSE file for details.

## Credits
- Inspired by ESP32 FM projects from Alexxdal and dg6rs.
- Developed by Emiliano Gonzalez.
