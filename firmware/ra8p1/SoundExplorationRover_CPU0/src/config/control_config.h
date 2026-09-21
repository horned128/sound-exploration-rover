/** =================================================================*
 * @file   control_config.h
 * @brief  CPU0の音源追従・障害物回避の調整値
 * ================================================================= */
#ifndef SEROV_CPU0_CONFIG_CONTROL_H
#define SEROV_CPU0_CONFIG_CONTROL_H


#define CPU0_SENSOR_CLEAR_DISTANCE_MM      (1000U)          /**< センサークリアのdistance[mm] */
#define CPU0_SENSOR_RECOVER_CLEAR_MM       (650U)           /**< センサー復帰のクリア[mm] */
#define CPU0_SENSOR_CAUTION_DISTANCE_MM    (700U)           /**< センサー注意のdistance[mm] */
#define CPU0_SENSOR_SIDE_DISTANCE_MM       (500U)           /**< センサー側面のdistance[mm] */
#define CPU0_SENSOR_PIVOT_DISTANCE_MM      (500U)           /**< センサーピボットのdistance[mm] */
#define CPU0_SENSOR_PIVOT_CLEAR_DISTANCE_MM (900U)          /**< センサーピボットクリアのdistance[mm] */
#define CPU0_SENSOR_HARD_STOP_DISTANCE_MM  (250U)           /**< センサーハード停止のdistance[mm] */
#define CPU0_SENSOR_CRITICAL_STOP_DISTANCE_MM (150U)        /**< センサー緊急停止のdistance[mm] */
/* 車体前端の突出と減速・測距遅延を見込み、後退で得た距離だけでは復帰しない。 */
#define CPU0_SENSOR_ESCAPE_SIDE_MM         (380U)           /**< センサー脱出の側面[mm] */
#define CPU0_SENSOR_ESCAPE_FRONT_MM        (550U)           /**< センサー脱出の正面[mm] */
#define CPU0_SENSOR_BACKUP_CLEAR_MM        (750U)           /**< センサー後退のクリア[mm] */
#define CPU0_SENSOR_BACKUP_GAIN_MM         (150U)           /**< センサー後退の余裕[mm] */
#define CPU0_SENSOR_SETTLE_MS              (400U)           /**< センサーの安定待ち[ms] */
#define CPU0_SENSOR_BACKUP_MIN_MS          (600U)           /**< センサー後退の最小[ms] */
#define CPU0_SENSOR_BACKUP_MAX_MS          (2500U)          /**< センサー後退の最大[ms] */
#define CPU0_SENSOR_PIVOT_MIN_MS           (600U)           /**< センサーピボットの最小[ms] */
#define CPU0_SENSOR_PIVOT_MAX_MS           (4500U)          /**< センサーピボットの最大[ms] */
#define CPU0_SENSOR_PIVOT_MIN_YAW_MDEG     (45000)          /**< センサーピボット最小のヨー[mdeg] */
#define CPU0_SENSOR_PIVOT_PROGRESS_MDEG    (5000)           /**< センサーピボットの進行[mdeg] */
#define CPU0_SENSOR_PIVOT_PROGRESS_MS      (1200U)          /**< センサーピボットの進行[ms] */
#define CPU0_SENSOR_CLEAR_HOLD_MS          (300U)           /**< センサークリアの保持[ms] */
#define CPU0_SENSOR_REARM_CLEAR_MS         (1000U)          /**< センサー再有効化のクリア[ms] */
#define CPU0_SENSOR_COMMIT_YAW_MDEG        (100000)         /**< センサー確定のヨー[mdeg] */
#define CPU0_SENSOR_COMMIT_STEERING_DEG    (30)             /**< センサー確定の操舵[deg] */
#define CPU0_SENSOR_COMMIT_MAX_MS          (10000U)         /**< センサー確定の最大[ms] */
#define CPU0_SENSOR_ESCAPE_MAX_ATTEMPTS    (3U)             /**< センサー脱出最大の試行回数 */
#define CPU0_SENSOR_MAX_STEP_MS            (500U)           /**< センサー最大の更新幅[ms] */
/* 車体中心の鉛直Z軸を使い、実機ログでは左旋回が負、右旋回が正となる。 */
#define CPU0_SENSOR_YAW_AXIS               (2U)             /**< 車体ヨーに使う鉛直Z軸 */
#define CPU0_SENSOR_YAW_RIGHT_SIGN         (1)              /**< センサーヨー右の符号 */
#define CPU0_SENSOR_YAW_DEADBAND_DPS_X10   (30)             /**< センサーヨーの不感帯[0.1dps] */
/*
 * 駆動系の実機校正値（始動境界 155‰、確実動作 200‰）に基づくRPM設定:
 * 実効Duty = RPM * (1000 / 300) * (800 / 1000) = RPM * 2.667‰
 * - 85 RPM  => 実効 227‰ (最低下限ガード: 200‰デッドバンドを確実に突破)
 * - 100 RPM => 実効 267‰ (注意減速時 / 後退時)
 * - 120 RPM => 実効 320‰ (通常前進 / ピボット外輪)
 */
#define CPU0_SENSOR_FORWARD_RPM            (120)            /**< センサーの前進[RPM] */
#define CPU0_SENSOR_CAUTION_RPM            (100)            /**< センサーの注意[RPM] */
#define CPU0_SENSOR_MIN_FORWARD_RPM        (85)             /**< センサー最小の前進[RPM] */
#define CPU0_SENSOR_BACKUP_RPM             (-100)           /**< センサーの後退[RPM] */
#define CPU0_SENSOR_PIVOT_OUTER_RPM        (120)            /**< センサーピボットの外輪[RPM] */
#define CPU0_SENSOR_PIVOT_INNER_RPM        (80)             /**< センサーピボットの内輪[RPM] */
#define CPU0_SENSOR_STEERING_MIN_DEG       (1)              /**< センサー操舵の最小[deg] */
#define CPU0_SENSOR_STEERING_MAX_DEG       (45)             /**< センサー操舵の最大[deg] */
#define CPU0_SENSOR_IMU_MAX_TILT_MG        (700)            /**< センサーIMU最大傾きのMG */
#define CPU0_SENSOR_IMU_MAX_SHOCK_L1_MG    (2400)           /**< センサーIMU最大衝撃L1のMG */
#define CPU0_SENSOR_IMU_MAX_GYRO_DPS_X10   (2000)           /**< センサーIMU最大の角速度[0.1dps] */

#define CPU0_SOUND_DOA_ZERO_OFFSET_DEG     (0)              /**< 音響DoAゼロの補正[deg] */
#define CPU0_SOUND_DOA_CLOCKWISE_POSITIVE  (0U)             /**< 音響DoA時計回りの正方向 */
#define CPU0_SOUND_TRIGGER_DBFS_X100       (-4500)          /**< 音響の開始[0.01dBFS] */
#define CPU0_SOUND_RELEASE_DBFS_X100       (-4800)          /**< 音響の解除[0.01dBFS] */
#define CPU0_SOUND_DOA_SETTLE_MS           (500U)           /**< 音響DoAの安定待ち[ms] */
#define CPU0_SOUND_DOA_ACQUIRE_TIMEOUT_MS  (2000U)          /**< 音響DoA取得の期限[ms] */
#define CPU0_SOUND_DOA_SAMPLE_COUNT        (5U)             /**< 音響DoAサンプルの個数 */
#define CPU0_SOUND_DOA_STABLE_WIDTH_DEG    (20)             /**< 音響DoA安定の幅[deg] */
#define CPU0_SOUND_FRONT_TOLERANCE_DEG     (0)              /**< 音響正面の許容幅[deg] */
#define CPU0_SOUND_STEERING_MIN_DEG        (1)              /**< 音響操舵の最小[deg] */
#define CPU0_SOUND_STEERING_MAX_DEG        (45)             /**< 音響操舵の最大[deg] */
#define CPU0_SOUND_SPIN_THRESHOLD_DEG      (90)             /**< 後方音源のその場旋回DoA[deg] */
#define CPU0_SOUND_SPIN_RPM                (300)            /**< その場旋回の左右車輪目標RPM絶対値 */
#define CPU0_SOUND_SPIN_SLOW_RPM           (220)            /**< その場旋回の終端減速RPM絶対値 */
#define CPU0_SOUND_SPIN_SERVO_DEG          (45)             /**< その場旋回の各舵輪角度絶対値[deg] */
#define CPU0_SOUND_SPIN_FRONT_RESERVE_DEG  (15)             /**< 前進操舵へ渡す残角[deg] */
#define CPU0_SOUND_SPIN_MAX_YAW_MDEG       (90000)          /**< その場旋回一回の最大目標ヨー[mdeg] */
#define CPU0_SOUND_SPIN_SLOWDOWN_MDEG      (20000)          /**< 終端減速を始める残ヨー[mdeg] */
#define CPU0_SOUND_SPIN_MAX_MS             (2200U)          /**< その場旋回の安全上限時間[ms] */
#define CPU0_SOUND_SPIN_PROGRESS_MS        (500U)           /**< その場旋回の進行を確認する時間[ms] */
#define CPU0_SOUND_SPIN_PROGRESS_MDEG      (3000)           /**< 進行成立とみなす最小ヨー角[mdeg] */
#define CPU0_SOUND_SPIN_FAILURE_HOLD_MS    (500U)           /**< 進行不足の通知保持[ms] */
#define CPU0_SOUND_LINK_STABLE_MS          (500U)           /**< 音響リンクの安定[ms] */
#define CPU0_SOUND_OBSERVATION_TIMEOUT_MS  (600U)           /**< 音響観測の期限[ms] */
#define CPU0_SOUND_STEER_SETTLE_MS         (500U)           /**< 音響操舵の安定待ち[ms] */
#define CPU0_SOUND_MOVE_STEP_MS            (1000U)          /**< 音響移動の更新幅[ms] */
#define CPU0_SOUND_LOST_TIMEOUT_MS         (1500U)          /**< 音響喪失の期限[ms] */
#define CPU0_SOUND_LISTEN_SETTLE_MS        (500U)           /**< 音響聴取の安定待ち[ms] */
#define CPU0_SOUND_COOLDOWN_RELEASE_MS     (200U)           /**< 音響クールダウンの解除[ms] */
#define CPU0_SOUND_MOVE_LEFT_RPM           (120)            /**< 音響移動の左[RPM] */
#define CPU0_SOUND_MOVE_RIGHT_RPM          (120)            /**< 音響移動の右[RPM] */
#define CPU0_SOUND_TURN_INNER_RPM          (90)             /**< 音響旋回の内輪[RPM] */
#define CPU0_STEERING_SERVO_OUTPUT_SIGN    (-1)             /**< 操舵サーボ出力の符号 */
/* 現場学習音響識別の走行反映: 0は従来互換（DoA+音量追従）、 */
/* 1は学習見本一致時のみ追従する。 */
#define CPU0_SOUND_REQUIRE_IDENTIFIER_MATCH (1U)            /**< 音響必須識別の一致 */
#define CPU0_SOUND_IDENTIFIER_TIMEOUT_MS   (1000U)          /**< 音響識別の期限[ms] */

#endif /* SEROV_CPU0_CONFIG_CONTROL_H */
