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

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audiosample_mono_8bit_8khz.h"
#include "esp32_tx.h"

#define FM_CARRIER_HZ 108000000UL //
#define MAX_DEV_HZ    75000UL     //
#define WAV_SR_HZ     8000UL      //
#define MOD_GAIN      2           // adjust to taste (8–32 typical)

static char TAG[] = "main";

void app_main() {
    tx_ctx_t tx_ctx = {
        .tx_cfg.carrier_hz = FM_CARRIER_HZ,                         //
        .tx_cfg.max_dev_hz = MAX_DEV_HZ,                            //
        .tx_cfg.wav_sr_hz = WAV_SR_HZ,                              //
        .tx_cfg.modulation_gain = MOD_GAIN,                         //
        .modulation.modulation_mode = MOD_FMW,                      //
        .modulation.filter_pre_hp = FILTER_HP_300_2pol,             //
        .modulation.filter_pre_lp = FILTER_LP_3400_4pol,            //
        .modulation.filter_pre_pb = FILTER_PB_NONE,                 //
        .modulation.filter_post_lp = FILTER_POST_LP_3400_4pol,      //
        .modulation.agc_type = AGC_NORMAL,                          //
        .modulation.special_modulation = SPECIAL_MODULATION_NORMAL, //
        .modulation.polar_status = 0,                               //
        .wav.audio = audio,                                         //
        .wav.audio_len = audio_len                                  //
    };

    polar_mod_init(&tx_ctx.polar_mod_ctx);

    ESP_LOGI(TAG, "fm_i2s_init");
    fm_i2s_init(tx_ctx);

    ESP_LOGI(TAG, "fm_route_to_pin");
    fm_route_to_pin();

    ESP_LOGI(TAG, "fm_apll_init");
    if (fm_apll_init(&tx_ctx)) {
        ESP_LOGI(TAG, "fm_start_audio");
        fm_start_audio(&tx_ctx);
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
