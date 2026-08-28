/** =================================================================*
 * @file   wifi_telemetry.h
 * @brief  Wi-Fiテレメトリー送信API
 * ================================================================= */
#ifndef RESPEAKER_WIFI_TELEMETRY_H
#define RESPEAKER_WIFI_TELEMETRY_H

#include "esp_err.h"                                        /* ESP-IDFエラー型 */
#include <stdbool.h>                                        /* 真偽値 */

esp_err_t wifi_telemetry_start(void);                       /* Wi-FiとUDP送信タスク開始 */
bool wifi_telemetry_is_connected(void);                     /* Wi-Fi接続状態取得 */

#endif /* RESPEAKER_WIFI_TELEMETRY_H */
