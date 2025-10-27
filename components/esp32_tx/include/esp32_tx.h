/*
 * Copyright 2025 Emiliano Gonzalez (egonzalez . hiperion @ gmail . com))
 * * Project Site: https://github.com/hiperiondev/esp32_fm_radio *
 *
 * This is based on other projects:
 *    ESP32 as FM radio transmitter: https://github.com/Alexxdal/ESP32FMRadio
 *    SSB/CW/FM signal generator 35 - 4400MHz: https://gitlab.com/dg6rs/polar
 *
 *    please contact their authors for more information.
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef ESP32_TX_H
#define ESP32_TX_H

#include <stdbool.h>
#include <stdint.h>

#include "polar_mod.h"
/**
 * @brief Audio PLL (APLL) configuration used for FM transmission.
 * Fields correspond to the APLL hardware registers / configuration used
 * to produce the requested output frequency. The fractional field
 * base_frac16 represents the 16-bit fractional part (sdm1:sdm0) and
 * dev_frac16 stores the number of fractional LSB steps corresponding
 * to the maximum frequency deviation requested.
 */
typedef struct {
    uint8_t o_div;        /**< Output divider (o_div) used by APLL */
    uint8_t sdm2;         /**< Integer part of APLL fractional multiplier (sdm2) */
    uint16_t base_frac16; /**< 16-bit fractional part (sdm1:sdm0) used as center */
    uint16_t dev_frac16;  /**< ±deviation expressed in same fractional units */
    bool is_rev0;         /**< true if chip revision is rev0 (affects APLL configuration) */
} apll_cfg_t;

typedef struct {
    uint32_t carrier_hz;
    uint32_t max_dev_hz;
    uint32_t wav_sr_hz;
    uint8_t modulation_gain;
} tx_cfg_t;

/**
 * @brief WAV to transmit
 */
typedef struct {
    const unsigned char *audio;
    const unsigned int audio_len;
} wav_t;

typedef struct {
    tx_cfg_t tx_cfg;
    apll_cfg_t apll_cfg;
    polar_mod_ctx_t polar_mod_ctx;
    modulation_t modulation;
    wav_t wav;
} tx_ctx_t;

/**
 * @brief Initialize and configure the I2S peripheral to use APLL as clock source.
 * This function sets up an I2S TX channel in master mode. The function configures
 * the clock source to I2S_CLK_SRC_APLL and requests the sample rate defined in the
 * implementation. The MCLK output is not assigned to a GPIO in this function (see
 * fm_route_to_pin()). After calling this, the I2S channel will be enabled and ready
 * to provide MCLK derived from APLL.
 *
 * @param tx_ctx Context
 */
void fm_i2s_init(tx_ctx_t tx_ctx);

/**
 * @brief Calculate and initialize the global APLL configuration and enable APLL.
 * This function computes sdm2, sdm1:sdm0 and o_div values that will generate the
 * requested carrier frequency taking into account the XTAL frequency and ensuring
 * the internal VCO stays within the valid lock range. It also programs the APLL
 * registers (via rtc_clk_apll_coeff_set) and enables the APLL.
 *
 * @param tx_ctx Context
 */
bool fm_apll_init(tx_ctx_t *tx_ctx);

/**
 * @brief Route the I2S MCLK (APLL derived) to a physical GPIO pin.
 * This maps the MCLK (CLK_OUT1) to GPIO0 and sets that GPIO as an output.
 * Note: GPIO0 is a strapping pin on many ESP32 modules; care must be taken
 * when using it as an RF output that the board can still boot normally.
 */
void fm_route_to_pin(void);

/**
 * @brief Start the periodic audio timer that performs real-time FM modulation.
 * The timer callback reads 8-bit PCM samples from the embedded audio array,
 * converts them to signed values, scales them by the precomputed deviation in
 * fractional LSB units and calls fm_set_deviation to update the APLL.
 *
 * @param tx_ctx Context
 */
void fm_start_audio(tx_ctx_t *tx_ctx);

#endif // ESP32_TX_H
