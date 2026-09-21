/** =================================================================*
 * @file   control_mlp_planner.h
 * @brief  TFLM int8制御MLPによる障害物回避プランナAPI
 * ================================================================= */
#ifndef SEROV_CPU0_CONTROL_MLP_PLANNER_H
#define SEROV_CPU0_CONTROL_MLP_PLANNER_H

#include "services/sensor_hub.h"                            /* センサースナップショット型 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型 */

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_MLP_INPUT_DIMENSION         (10U)           /**< 入力テンソル次元数 */
#define CONTROL_MLP_OUTPUT_DIMENSION        (2U)            /**< 出力テンソル次元数 */
#define CONTROL_MLP_MAX_STEERING_DEG        (45.0f)         /**< 最大操舵角 [deg] */
#define CONTROL_MLP_CRITICAL_DISTANCE_MM    (150.0f)        /**< 正面衝突判定はルール制御側で実施 [mm] */
#define CONTROL_MLP_FAR_DISTANCE_MM         (4000.0f)       /**< 無効時最大距離 [mm] */
#define CONTROL_MLP_MAX_STEER_RATE_DPS      (90.0f)         /**< 最大操舵角変化率 [deg/s] */
#define CONTROL_MLP_MAX_ACCEL_PER_SEC       (1.5f)          /**< 最大速度スケール変化率 [/s] */
#define CONTROL_MLP_STEP_DT_SEC             (0.10f)         /**< 思考タスク周期時間 [s] */

/**< 制御MLPプランナ出力 */
typedef struct st_control_mlp_output {
    float steering_deg;                                     /**< 決定操舵角 [-45, +45] */
    float speed_scale;                                      /**< 決定速度スケール [0.0, 1.0] */
    BOOL  is_blocked;                                       /**< 推論結果が停止スケールの状態 */
    BOOL  emergency_stop;                                   /**< 非常停止要求 */
    BOOL  fallback_required;                                /**< ルールベースフォールバック要求 */
} control_mlp_output_t;

EXPORT BOOL control_mlp_planner_init(void);                 /* TFLMモデル初期化 */
EXPORT void control_mlp_planner_step(const sensor_snapshot_t * p_snapshot,
                                     float target_heading_deg,
                                     control_mlp_output_t * p_output); /* 回避計画算出 */
EXPORT void control_mlp_planner_reset(void);                /* プランナ状態リセット */
EXPORT BOOL control_mlp_planner_is_ready(void);             /* 初期化状態取得 */

#ifdef __cplusplus
}
#endif

#endif /* SEROV_CPU0_CONTROL_MLP_PLANNER_H */
