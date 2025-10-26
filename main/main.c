#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audiosample_mono_8bit_8khz.h"
#include "fm_tx.h"

#define FM_CARRIER_HZ 108000000UL //
#define MAX_DEV_HZ    75000UL     //
#define WAV_SR_HZ     8000UL      //

static char TAG[] = "main";

void app_main() {
    tx_ctx_t tx_ctx = {
        .tx_cfg.fm_carrier_hz = FM_CARRIER_HZ, //
        .tx_cfg.max_dev_hz = MAX_DEV_HZ,       //
        .tx_cfg.wav_sr_hz = WAV_SR_HZ,         //
        .wav.audio = rickroll,                 //
        .wav.audio_len = rickroll_len          //
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
