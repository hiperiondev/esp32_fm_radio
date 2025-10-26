#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audiosample_mono_8bit_8khz.h"
#include "fm_tx.h"

#define FM_CARRIER_HZ 108000000UL //
#define MAX_DEV_HZ    75000UL     //
#define WAV_SR_HZ     8000UL      //
#define MOD_GAIN      2           // adjust to taste (8–32 typical)

static char TAG[] = "main";

void app_main() {
    tx_ctx_t tx_ctx = {
        .tx_cfg.fm_carrier_hz = FM_CARRIER_HZ,                      //
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
        .wav.audio = rickroll,                                      //
        .wav.audio_len = rickroll_len                               //
    };

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
