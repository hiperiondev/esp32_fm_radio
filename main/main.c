#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "fm_tx.h"
#include "rickroll.h"

#define FM_CARRIER_HZ 100000000UL
#define MAX_DEV_HZ    12500UL
#define WAV_SR_HZ     8000UL

static char TAG[] = "main";

void app_main() {
    wav_t wav_file = { .audio = rickroll, .audio_len = rickroll_len };

    ESP_LOGI(TAG, "fm_i2s_init");
    fm_i2s_init(WAV_SR_HZ);

    ESP_LOGI(TAG, "fm_route_to_pin");
    fm_route_to_pin();

    ESP_LOGI(TAG, "fm_apll_init");
    if (fm_apll_init(FM_CARRIER_HZ, MAX_DEV_HZ)) {
        ESP_LOGI(TAG, "fm_start_audio");
        fm_start_audio(WAV_SR_HZ, &wav_file);
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
