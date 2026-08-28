#include "acoustic_frontend.h"

#include "audio_capture.h"
#include "esp_check.h"
#include "usb_link.h"
#include "wifi_telemetry.h"
#include "xvf3800_control.h"

void app_main(void) {
    ESP_ERROR_CHECK(xvf3800_control_init());
    ESP_ERROR_CHECK(audio_capture_start());
    ESP_ERROR_CHECK(usb_link_init());
    (void) wifi_telemetry_start();
    ESP_ERROR_CHECK(acoustic_frontend_start());
}
