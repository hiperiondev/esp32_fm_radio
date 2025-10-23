#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hal/clk_tree_ll.h"
#include "hal/efuse_ll.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc.h"
#include "soc/soc.h"

#include "fm_tx.h"

#if !CONFIG_IDF_TARGET_ESP32
#error "This project requires a chip with APLL (ESP32 D0WD / WROOM / WROVER). The S2/S3/C3 series do not have it."
#endif

static const char *TAG = "fm_tx";
static fm_apll_cfg_t g_apll;
static i2s_chan_handle_t tx_handle;

/**
 * @brief Return the chip's crystal oscillator frequency in Hz.
 *
 * Uses the RTC helper rtc_clk_xtal_freq_get() which returns MHz; convert to Hz.
 *
 * @return XTAL frequency in Hz (e.g., 40000000 for a 40 MHz crystal)
 */
static inline uint32_t get_xtal_hz(void) {
    // get xtal in MHz and convert to Hz
    return rtc_clk_xtal_freq_get() * 1000000UL;
}

/**
 * @brief Compute APLL configuration parameters for a requested output and deviation.
 *
 * This routine finds the smallest o_div such that the internal VCO frequency
 * (fout * 2 * (o_div + 2)) is >= 350 MHz (APLL lock constraint). It then
 * computes sdm2 and the 16-bit fractional component (base_frac16) to produce
 * the requested output frequency given the XTAL. It also computes dev_frac16
 * as the number of fractional steps corresponding to dev_hz so samples may
 * be scaled and applied in those units.
 *
 * The function will slightly shift base_frac16 if it would otherwise be too
 * close to 0 or 65535 so symmetrical +/- deviation is possible.
 *
 * @param fout_hz Desired APLL output (carrier) in Hz
 * @param dev_hz Maximum desired frequency deviation in Hz (absolute)
 * @return Populated fm_apll_cfg_t structure
 */
static fm_apll_cfg_t fm_calc_apll(uint32_t fout_hz, uint32_t dev_hz) {
    fm_apll_cfg_t best = { 0 };
    uint64_t XTAL = (uint64_t)get_xtal_hz();
    if (XTAL == 0) {
        ESP_LOGI(TAG, "fm_calc_apll_opt: XTAL==0, returning zeroed config");
        best.is_rev0 = (efuse_ll_get_chip_ver_rev1() == 0);
        return best;
    }

    const uint64_t VCO_MIN = 350000000ULL;
    const uint64_t VCO_MAX = 500000000ULL;

    bool have_best = false;
    uint64_t best_err = UINT64_MAX;

    bool prefer_inrange = true;
    fm_apll_cfg_t best_inrange = { 0 };
    bool have_inrange = false;
    uint64_t best_err_inrange = UINT64_MAX;

    // try all allowed output dividers (o_div)
    for (uint32_t o_div = 0; o_div <= 3; ++o_div) {
        uint64_t o_mul = (uint64_t)(o_div + 2);

        // compute total multiplier in 16.16 fixed point
        uint64_t numer = (uint64_t)fout_hz * 2ULL * o_mul * 65536ULL;
        uint64_t denom = XTAL;
        uint64_t total_mul16 = (numer + denom / 2ULL) / denom;

        int64_t sdm2_i = (int64_t)(total_mul16 >> 16) - 4;
        if (sdm2_i < 0 || sdm2_i > 63) {
            continue;
        }

        uint8_t sdm2 = (uint8_t)sdm2_i;
        uint16_t base_frac16 = (uint16_t)(total_mul16 & 0xFFFFU);

        // produced frequency
        uint64_t produced_mul16 = ((uint64_t)(4 + sdm2) << 16) | (uint64_t)base_frac16;
        uint64_t produced_num = XTAL * produced_mul16;
        uint64_t produced_den = o_mul * 2ULL * 65536ULL;
        uint64_t produced_freq_hz = (produced_num + produced_den / 2ULL) / produced_den;

        uint64_t err = (produced_freq_hz > fout_hz) ? (produced_freq_hz - fout_hz) : (fout_hz - produced_freq_hz);

        // compute fractional delta for deviation
        uint64_t dev_numer = (uint64_t)dev_hz * 2ULL * o_mul * 65536ULL;
        uint64_t dev_denom = XTAL;
        uint64_t dev_frac = (dev_numer + dev_denom / 2ULL) / dev_denom;
        if (dev_frac > 0xFFFFULL)
            dev_frac = 0xFFFFULL;
        uint16_t dev_frac16 = (uint16_t)dev_frac;

        // update absolute best
        if (!have_best || err < best_err) {
            have_best = true;
            best_err = err;
            best.o_div = (uint8_t)o_div;
            best.sdm2 = sdm2;
            best.base_frac16 = base_frac16;
            best.dev_frac16 = dev_frac16;
        }

        // update in-range best
        uint64_t vco = (uint64_t)fout_hz * 2ULL * o_mul;
        if (vco >= VCO_MIN && vco <= VCO_MAX) {
            if (!have_inrange || err < best_err_inrange) {
                have_inrange = true;
                best_inrange.o_div = (uint8_t)o_div;
                best_inrange.sdm2 = sdm2;
                best_inrange.base_frac16 = base_frac16;
                best_inrange.dev_frac16 = dev_frac16;
                best_err_inrange = err; // track separate in-range error
            }
        }
    }

    fm_apll_cfg_t chosen;
    if (prefer_inrange && have_inrange) {
        chosen = best_inrange;
    } else if (have_best) {
        chosen = best;
    } else {
        chosen = (fm_apll_cfg_t){ 0 };
        chosen.is_rev0 = (efuse_ll_get_chip_ver_rev1() == 0);
        ESP_LOGI(TAG, "no valid APLL candidate found");
        return chosen;
    }

    // Adjust base_frac16 only when sdm2 is at 0 or 63 and ±dev_frac16 would overflow/underflow beyond the valid range 0..63.
    if (chosen.dev_frac16 != 0) {
        if (chosen.sdm2 == 0 && chosen.base_frac16 < chosen.dev_frac16) {
            chosen.base_frac16 = chosen.dev_frac16;
        } else if (chosen.sdm2 == 63 && chosen.base_frac16 > (uint16_t)(65535U - chosen.dev_frac16)) {
            chosen.base_frac16 = (uint16_t)(65535U - chosen.dev_frac16);
        }
    }

    chosen.is_rev0 = (efuse_ll_get_chip_ver_rev1() == 0);

    return chosen;
}

/**
 * @brief Apply a signed fractional deviation to the currently configured APLL.
 *
 * This routine takes delta_frac16, adds it to the stored base fractional value
 * and manages carry/borrow into sdm2 (the integer portion) so the combined
 * sdm2 + fractional value is a valid representation. It clamps the resulting
 * sdm2 and fractional value to the allowed hardware ranges and then calls the
 * low-level clock driver to update APLL config immediately.
 *
 * @param delta_frac16 Signed deviation in 1/65536 fractional units to apply
 */
static inline void fm_set_deviation(int16_t delta_frac16) {
    int32_t frac32 = (int32_t)g_apll.base_frac16 + (int32_t)delta_frac16; // fractional accumulator
    int32_t sdm2 = (int32_t)g_apll.sdm2;                                  // integer part

    // borrow if fractional part underflows
    if (frac32 < 0) {
        int32_t borrow = (-frac32 + 65535) >> 16; // number of 65536 wraps needed
        frac32 += borrow * 65536;
        sdm2 -= borrow; // decrement integer part accordingly
    } else if (frac32 > 65535) {
        int32_t carry = frac32 >> 16; // number of integer increments
        frac32 -= carry * 65536;
        sdm2 += carry; // increment integer part accordingly
    }

    // clamp to hardware-supported ranges
    if (sdm2 < 0) {
        sdm2 = 0;
        frac32 = 0;
    }
    if (sdm2 > 63) {
        sdm2 = 63;
        frac32 = 65535;
    }

    // split into register-friendly bytes
    uint8_t sdm0 = (uint8_t)(frac32 & 0xFF);
    uint8_t sdm1 = (uint8_t)((frac32 >> 8) & 0xFF);
    uint8_t sdm2_u8 = (uint8_t)sdm2;

    // update APLL config via low-level clock driver
    clk_ll_apll_set_config(g_apll.is_rev0, (uint8_t)g_apll.o_div, sdm0, sdm1, sdm2_u8);
}

/**
 * @brief Timer callback executed at the audio sample rate to update APLL deviation.
 *
 * The ISR reads the next sample from the embedded 8-bit PCM array, converts it
 * to signed form (-128..127), scales it by the precomputed dev_frac16 and calls
 * fm_set_deviation() to update the APLL fractional registers. This callback is
 * marked IRAM_ATTR to ensure it runs from instruction RAM for timing consistency.
 *
 * @param arg Unused (user arg passed by esp_timer, ignored)
 */
static void IRAM_ATTR fm_timer_cb(void *arg) {
    wav_t *wav = (wav_t *)arg;
    static size_t pos = 0;                            // position in embedded audio array
    int16_t audio = (int16_t)wav->audio[pos++] - 128; // convert unsigned->signed
    if (pos >= wav->audio_len)
        pos = 0; // loop the audio

    // scale signed audio to fractional LSB units and update APLL
    int16_t delta = (int16_t)(((int32_t)audio * (int32_t)g_apll.dev_frac16) >> 7);
    fm_set_deviation(delta);
}

/**
 * @brief Route the I2S-derived MCLK to GPIO0 (CLK_OUT1) and set it as output.
 *
 * After calling this, the selected GPIO will present the MCLK output as a
 * continuous digital clock signal derived from the APLL when the I2S driver
 * has been configured and enabled.
 */
void fm_route_to_pin(void) {
    // select CLK_OUT1 function for GPIO0
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    // set source for CLK_OUT1 to I2S0 MCLK
    REG_SET_FIELD(PIN_CTRL, CLK_OUT1, 0);
    // mark GPIO0 as output
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "MCLK routed to GPIO0 (CLK_OUT1)");
}

/**
 * @brief Initialize I2S in standard TX master mode using APLL as clock source.
 *
 * The implementation creates a new I2S channel and configures it for the
 * sample rate used by the embedded waveform. The MCLK routing in this build
 * intentionally leaves the MCLK GPIO unassigned (fm_route_to_pin handles
 * mapping to a physical pin). The chosen mclk_multiple constant is selected
 * so the driver will use APLL and produce an appropriate MCLK frequency.
 *
 * @param wav_sr_hz Sample rate
 */
void fm_i2s_init(uint32_t wav_sr_hz) {
    // create new I2S channel in master role
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    // configure standard I2S settings; note mclk_multiple requests high-speed MCLK
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = wav_sr_hz, // audio sample rate defines timer rate
            .clk_src = I2S_CLK_SRC_APLL, // use audio PLL
            .mclk_multiple = I2S_MCLK_MULTIPLE_512 // request large MCLK multiple
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = GPIO_NUM_NC,
            .ws   = GPIO_NUM_NC,
            .dout = GPIO_NUM_NC,
            .din  = GPIO_NUM_NC
        },
    };

    // initialize and enable the channel
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    ESP_LOGI(TAG, "I2S STD channel started (APLL source, %u Hz sample rate)", wav_sr_hz);
}

/**
 * @brief Initialize and program the APLL registers for the chosen carrier and deviation.
 *
 * This routine computes the APLL constants using fm_calc_apll(), then programs
 * the rtc APLL registers and enables the APLL so the hardware begins generating
 * the requested clock. The base fractional bytes are split into sdm0/sdm1 for
 * rtc_clk_apll_coeff_set.
 * 
 * @param fm_carrier_hz Desired APLL output (carrier) in Hz
 * @param max_dev_hz Maximum desired frequency deviation in Hz (absolute)
 */
bool fm_apll_init(uint32_t fm_carrier_hz, uint32_t max_dev_hz) {
    uint32_t min_carrier = get_xtal_hz() * 2 / 5;
    if (fm_carrier_hz < min_carrier || fm_carrier_hz > 125000000) {
        ESP_LOGI(TAG, "Carrier out of range [%uMhz, 125MHz]: %0.2fMHz", min_carrier / 1000000, fm_carrier_hz / 1000000.0);
        return false;
    }

    // Compute APLL configuration for target carrier and deviation
    g_apll = fm_calc_apll(fm_carrier_hz, max_dev_hz);

    // Extract fractional parts for hardware registers
    uint8_t sdm0 = (uint8_t)(g_apll.base_frac16 & 0xFF);
    uint8_t sdm1 = (uint8_t)((g_apll.base_frac16 >> 8) & 0xFF);

    // Enable APLL and set coefficients
    rtc_clk_apll_enable(true);
    rtc_clk_apll_coeff_set(g_apll.o_div, sdm0, sdm1, g_apll.sdm2);

    // Compute produced frequency from register configuration
    // Formula from ESP32 Technical Reference Manual (TRM):
    // f_out = XTAL * (4 + sdm2 + sdm1/256 + sdm0/65536) / (2 * (o_div + 2))
    double fout_hz = (double)get_xtal_hz() * (4.0 + (double)g_apll.sdm2 + (double)sdm1 / 256.0 + (double)sdm0 / 65536.0) / (2.0 * (g_apll.o_div + 2));

    // Calculate frequency error in Hz (signed)
    double error_hz = fout_hz - (double)fm_carrier_hz;

    ESP_LOGI(TAG, "APLL configured: XTAL=%u Hz target=%.2f Hz produced=%.2f Hz error=%.2f Hz (o_div=%u sdm2=%u sdm1=%u sdm0=%u base_frac=%u dev_frac=%u)",
             get_xtal_hz(), (double)fm_carrier_hz, fout_hz, error_hz, (unsigned)g_apll.o_div, (unsigned)g_apll.sdm2, (unsigned)sdm1, (unsigned)sdm0,
             (unsigned)g_apll.base_frac16, (unsigned)g_apll.dev_frac16);

    return true;
}

/**
 * @brief Start a periodic high-resolution timer to drive audio modulation.
 *
 * The function creates an esp_timer configured with the callback above and
 * starts it in periodic mode with period derived from the WAV sample rate.
 *
 * @param wav_sr_hz Sample rate
 * @param wav_file Wav data
 */
void fm_start_audio(uint32_t wav_sr_hz, wav_t *wav_file) {
    const esp_timer_create_args_t args = { .callback = &fm_timer_cb, .name = "fm_audio_timer", .arg = (void *)wav_file };
    esp_timer_handle_t timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
    const uint64_t period_us = 1000000ULL / wav_sr_hz;
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, period_us));
    ESP_LOGI(TAG, "Audio timer started at %u Hz", wav_sr_hz);
}
