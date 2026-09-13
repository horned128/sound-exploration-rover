/** =================================================================*
 * @file   sensor_liveness.h
 * @brief  取得側から独立したセンサー更新監視
 * ================================================================= */
#ifndef SEROV_CPU0_SENSOR_LIVENESS_H
#define SEROV_CPU0_SENSOR_LIVENESS_H
#include <tk/tkernel.h>                                    /* 基本型 */

typedef struct st_sensor_liveness {
    UD last_seen_ms;
    UD last_progress_ms;
    UW last_count;
    UW age_ms;
    BOOL initialized;
    BOOL progress_seen;
} sensor_liveness_t;

/** =================================================================*
 * @brief  更新回数の進行と観測側の実時間による鮮度判定
 * @details 初回の値だけでは進行を証明できないため、次の更新まで不許可。
 *          カウンタ折返しは不一致で検出し、時刻異常・取得失敗時は再確認を要求する。
 * @param[in,out] p_state 観測側専有の監視状態
 * @param[in] count 取得側の正常更新回数
 * @param[in] now_ms 観測側の単調増加時刻[ms]
 * @param[in] available 時刻と初期化済みsnapshotの取得成功
 * @param[in] timeout_ms 更新停止の判定期限[ms]
 * @return 更新進行を観測し、期限内ならTRUE
 * ================================================================= */
Inline BOOL sensor_liveness_update(sensor_liveness_t * p_state, UW count, UD now_ms,
                                   BOOL available, UW timeout_ms) {
    if (!available || (p_state->initialized && (now_ms < p_state->last_seen_ms))) {
        *p_state = (sensor_liveness_t){.age_ms = UINT32_MAX};
        return FALSE;
    }
    if (!p_state->initialized) {
        *p_state = (sensor_liveness_t){
            .last_seen_ms = now_ms, .last_progress_ms = now_ms, .last_count = count,
            .age_ms = UINT32_MAX, .initialized = TRUE, .progress_seen = FALSE,
        };
        return FALSE;
    }
    p_state->last_seen_ms = now_ms;
    if (count != p_state->last_count) {
        p_state->last_count = count;
        p_state->last_progress_ms = now_ms;
        p_state->progress_seen = TRUE;
    }
    UD const elapsed = now_ms - p_state->last_progress_ms;
    p_state->age_ms = (!p_state->progress_seen || (elapsed > UINT32_MAX)) ? UINT32_MAX : (UW) elapsed;
    return p_state->progress_seen && (elapsed < timeout_ms);
}
#endif /* SEROV_CPU0_SENSOR_LIVENESS_H */
