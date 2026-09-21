/** =================================================================*
 * @file   sound_follow_controller.h
 * @brief  音源追従行動生成
 * ================================================================= */
#ifndef SEROV_CPU0_SOUND_FOLLOW_CONTROLLER_H
#define SEROV_CPU0_SOUND_FOLLOW_CONTROLLER_H

#include "../../../common/acoustic_protocol.h"              /* 音響観測型 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

/**< 音源追従とセンサー回避の制御状態 */
typedef enum e_sound_follow_state {
    CPU0_THINK_STATE_WAIT_LINK = 0,                         /**< 音響リンク待ち */
    CPU0_THINK_STATE_LISTEN,                                /**< 音源聴取中 */
    CPU0_THINK_STATE_STEER_PREP,                            /**< 操舵準備中 */
    CPU0_THINK_STATE_MOVE_STEP,                             /**< 走行ステップ中 */
    CPU0_THINK_STATE_SETTLE,                                /**< 停止・整定中 */
    CPU0_THINK_STATE_COOLDOWN,                              /**< 再検出待ち */
    CPU0_THINK_STATE_SENSOR_SAFE_STOP,                      /**< センサー安全停止 */
    CPU0_THINK_STATE_SENSOR_FORWARD,                        /**< センサー判断の前進 */
    CPU0_THINK_STATE_SENSOR_CAUTION_FORWARD,                /**< センサー判断の注意前進 */
    CPU0_THINK_STATE_SENSOR_TURN_LEFT,                      /**< センサー判断の左旋回 */
    CPU0_THINK_STATE_SENSOR_TURN_RIGHT,                     /**< センサー判断の右旋回 */
    CPU0_THINK_STATE_SENSOR_BLOCKED_STOP,                   /**< 障害物による停止 */
    CPU0_THINK_STATE_SENSOR_IMU_STOP,                       /**< IMU異常による停止 */
    CPU0_THINK_STATE_FAULT,                                 /**< システム異常停止 */
    CPU0_THINK_STATE_SENSOR_PIVOT_LEFT,                     /**< センサー判断の左ピボット */
    CPU0_THINK_STATE_SENSOR_PIVOT_RIGHT,                    /**< センサー判断の右ピボット */
    CPU0_THINK_STATE_SENSOR_BACKUP,                         /**< 予約済み旧自律後退状態（未使用） */
    CPU0_THINK_STATE_SPIN_PREP,                             /**< 最小並進回頭の舵角整定中 */
    CPU0_THINK_STATE_SPIN_STEP,                             /**< 最小並進回頭中 */
    CPU0_THINK_STATE_SPIN_NO_PROGRESS,                      /**< 最小並進回頭のヨー進行不足停止 */
    CPU0_THINK_STATE_ARRIVAL_VERIFY,                        /**< 音源到着候補の停止確認 */
    CPU0_THINK_STATE_ARRIVED,                               /**< 音源到着による停止 */
} sound_follow_state_t;

/**< 音源追従ステートマシンへ入力するリンク・安全・照合状態 */
typedef struct st_sound_follow_input {
    BOOL link_ready;                                        /**< 音響リンク準備完了状態 */
    BOOL new_observation;                                   /**< 新しい音響観測の到着状態 */
    BOOL fault_active;                                      /**< システム異常の発生状態 */
    BOOL motion_allowed;                                    /**< 安全調停による走行許可 */
    acoustic_observation_t observation;                     /**< 最新音響観測 */
    BOOL match_required;                                    /**< 見本照合を要求する状態 */
    BOOL target_sound_matched;                              /**< 対象音一致状態 */
    BOOL navigation_target_valid;                           /**< 位置推定済み音源目標が有効 */
    H navigation_bearing_deg;                               /**< 現在位置から音源への方位（右正）[deg] */
    BOOL arrival_verify;                                    /**< 到着候補の停止確認要求 */
    BOOL arrived;                                           /**< 音源到着停止要求 */
    BOOL imu_valid;                                         /**< 生ジャイロZ値を利用可能な状態 */
    H gyro_z_dps_x10;                                       /**< 生ジャイロZ角速度[0.1dps] */
    UW imu_update_count;                                    /**< センサーの正常更新回数 */
} sound_follow_input_t;

/**< 音源追従ステートマシンが出力する走行指令 */
typedef struct st_sound_follow_output {
    sound_follow_state_t state;                             /**< 追従状態 */
    H steering_deg;                                         /**< 操舵角指令[deg] */
    BOOL is_spin_turn;                                      /**< X字操舵による最小並進回頭指令 */
    H left_rpm;                                             /**< 左車輪指令RPM */
    H right_rpm;                                            /**< 右車輪指令RPM */
    BOOL actuator_enable;                                   /**< アクチュエータ出力許可 */
    BOOL emergency_stop;                                    /**< 非常停止指令 */
} sound_follow_output_t;

EXPORT void sound_follow_controller_init(void);             /* 追従状態初期化 */
EXPORT H sound_follow_doa_to_relative(UH doa_deg);          /* XVF DoAから車体相対角への変換 */
EXPORT void sound_follow_controller_step(const sound_follow_input_t * p_input,
                                  UW elapsed_ms,
                                  sound_follow_output_t * p_output); /* 追従状態更新 */

#endif /* SEROV_CPU0_SOUND_FOLLOW_CONTROLLER_H */
