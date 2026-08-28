/** =================================================================*
 * @file   app_main.c
 * @brief  ESP32S3アプリケーション起動
 * ================================================================= */
#include "acoustic_frontend.h"                              /* 音響観測フロントエンド起動API */

#include "audio_capture.h"                                  /* 音声キャプチャ起動API */
#include "esp_check.h"                                      /* ESP-IDFエラー検査マクロ */
#include "usb_link.h"                                       /* USB CDC通信初期化API */
#include "wifi_telemetry.h"                                 /* Wi-Fiテレメトリー起動API */
#include "xvf3800_control.h"                                /* XVF3800初期化API */

/** =================================================================*
 * @brief  ESP-IDFアプリケーション開始
 * ================================================================= */
void app_main(void) {
    ESP_ERROR_CHECK(xvf3800_control_init());
    ESP_ERROR_CHECK(audio_capture_start());
    ESP_ERROR_CHECK(usb_link_init());
    (void) wifi_telemetry_start();
    ESP_ERROR_CHECK(acoustic_frontend_start());
}
