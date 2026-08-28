/** =================================================================*
 * @file   hal_entry.c
 * @brief  CPU1アプリケーション入口
 * ================================================================= */
#include "app/actuator_app.h"                               /* CPU1アクチュエータアプリケーションAPI */
#include "cpu1_config.h"                                    /* CPU1アクチュエータ設定 */
#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンスと型定義 */

extern bsp_leds_t g_bsp_leds;                               /**< BSPが管理するLED構成情報 */

#define CPU1_STATUS_LED_INDEX (2U)                          /* CPU1状態表示に使うLED番号 */

/** =================================================================*
 * @brief  CPU1アプリケーション実行
 * @details FSP初期化後、IPC経由のアクチュエータ処理を実行する。
 * ================================================================= */
void hal_entry(void) {
    bsp_leds_t const leds = g_bsp_leds;
    bsp_io_level_t led_level = BSP_IO_LEVEL_LOW;
    uint32_t heartbeat_ms = 0U;

#if BSP_TZ_SECURE_BUILD
    R_BSP_NonSecureEnter();
#endif

    (void) actuator_app_init();

    while (1) {
        actuator_app_run_1ms();

        heartbeat_ms++;
        uint32_t const heartbeat_period_ms = (FSP_SUCCESS == g_actuator_last_error) ? 500U : 50U;
        if ((leds.led_count > CPU1_STATUS_LED_INDEX) && (heartbeat_ms >= heartbeat_period_ms)) {
            (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, leds.p_leds[CPU1_STATUS_LED_INDEX], led_level);
            led_level = (BSP_IO_LEVEL_LOW == led_level) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;
            heartbeat_ms = 0U;
        }

        R_BSP_SoftwareDelay(ACTUATOR_LOOP_PERIOD_MS, BSP_DELAY_UNITS_MILLISECONDS);
    }
}
