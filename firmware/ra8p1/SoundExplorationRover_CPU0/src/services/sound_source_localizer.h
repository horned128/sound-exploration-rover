/** =================================================================*
 * @file   sound_source_localizer.h
 * @brief  複数DoAとオドメトリを用いたbearing-only音源位置推定
 * ================================================================= */
#ifndef SEROV_CPU0_SERVICE_SOUND_SOURCE_LOCALIZER_H
#define SEROV_CPU0_SERVICE_SOUND_SOURCE_LOCALIZER_H

#include "services/odometry.h"                             /* 車体位置姿勢 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型 */

/**< 音源到着判定の段階。障害物停止・非常停止とは独立する。 */
typedef enum e_sound_arrival_state {
    CPU0_SOUND_ARRIVAL_SEARCH = 0,                          /**< 位置未確定 */
    CPU0_SOUND_ARRIVAL_TRACKING,                            /**< 有効な音源位置を追跡中 */
    CPU0_SOUND_ARRIVAL_VERIFY,                              /**< 到着候補の継続確認中 */
    CPU0_SOUND_ARRIVAL_ARRIVED,                             /**< 音源へ到着し停止中 */
} sound_arrival_state_t;

/**< 位置推定器への周期入力 */
typedef struct st_sound_source_localizer_input {
    odometry_pose_t pose;                                   /**< 現在の車体位置姿勢 */
    H relative_doa_deg;                                     /**< 車体正面基準DoA（右正）[deg] */
    UB doa_confidence;                                      /**< DoA品質[0..100] */
    BOOL new_observation;                                   /**< 未処理の新規観測 */
    BOOL sound_valid;                                       /**< VAD・音量・識別を通過した対象音 */
    BOOL pose_valid;                                        /**< poseが同期済みかつfresh */
    UW observation_sequence;                               /**< DoA観測専用sequence */
    UW now_ms;                                              /**< 単調時刻[ms] */
} sound_source_localizer_input_t;

/**< 位置推定・到着判定の診断付き出力 */
typedef struct st_sound_source_localizer_output {
    W source_x_mm;                                          /**< 推定音源X座標[mm] */
    W source_y_mm;                                          /**< 推定音源Y座標[mm] */
    UW source_range_mm;                                     /**< 現在位置からの音源距離[mm] */
    H source_bearing_deg;                                   /**< 車体正面からの目標方位（右正）[deg] */
    UB source_confidence;                                   /**< 音源位置品質[0..100] */
    UB observation_count;                                   /**< 使用中の方位観測数 */
    BOOL source_position_valid;                             /**< 幾何条件を満たす位置推定あり */
    BOOL localization_geometry_valid;                       /**< 今回の最小二乗幾何が有効 */
    BOOL navigation_target_valid;                           /**< 追従に使える保持期限内目標 */
    BOOL arrival_candidate;                                /**< 距離・品質・音の到着候補 */
    UH localization_residual_mm;                            /**< 方位線残差RMS[mm] */
    UH bearing_crossing_angle_deg;                          /**< 最大方位交差角[deg] */
    UH baseline_mm;                                         /**< 最大観測基線[mm] */
    UH source_position_shift_mm;                            /**< 前回有効推定からの変化[mm] */
    UB arrival_confirm_count;                               /**< 到着確認済み観測数 */
    sound_arrival_state_t arrival_state;                    /**< 到着判定段階 */
} sound_source_localizer_output_t;

EXPORT void sound_source_localizer_init(void);              /* 推定履歴初期化 */
EXPORT void sound_source_localizer_step(const sound_source_localizer_input_t * p_input,
                                        sound_source_localizer_output_t * p_output); /* 推定更新 */

#endif /* SEROV_CPU0_SERVICE_SOUND_SOURCE_LOCALIZER_H */
