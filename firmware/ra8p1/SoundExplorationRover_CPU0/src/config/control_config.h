/** =================================================================*
 * @file   control_config.h
 * @brief  CPU0の音源追従・障害物回避の調整値
 * ================================================================= */
#ifndef SEROV_CPU0_CONFIG_CONTROL_H
#define SEROV_CPU0_CONFIG_CONTROL_H


#define CPU0_SENSOR_CAUTION_DISTANCE_MM    (700U)           /**< センサー注意のdistance[mm] */
#define CPU0_SENSOR_EARLY_AVOID_DISTANCE_MM (800U)          /**< 片側が開いた正面障害物の先行回避距離[mm] */
#define CPU0_SENSOR_EARLY_AVOID_SIDE_DELTA_MM (350U)        /**< 先行回避に必要な左右差[mm] */
#define CPU0_SENSOR_AVOID_CLEAR_DISTANCE_MM (900U)          /**< 回避側を解除する正面距離[mm] */
#define CPU0_SENSOR_AVOID_CLEAR_SIDE_MM    (600U)           /**< 回避側を解除する側方距離[mm] */
#define CPU0_SENSOR_PIVOT_DISTANCE_MM      (500U)           /**< センサーピボットのdistance[mm] */
#define CPU0_SENSOR_BACKUP_TRIGGER_DISTANCE_MM (300U)       /**< 壁面後退を始める正面距離[mm] */
#define CPU0_SENSOR_COLLISION_CONFIRM_COUNT (2U)            /**< 後退を始める正面近接の連続確認回数 */
/* 車体前端の突出と旋回掃引を見込み、500 mmで最小並進旋回を開始する。 */
#define CPU0_SENSOR_ESCAPE_SIDE_MM         (380U)           /**< センサー脱出の側面[mm] */
#define CPU0_SENSOR_ESCAPE_FRONT_MM        (550U)           /**< センサー脱出の正面[mm] */
#define CPU0_SENSOR_SETTLE_MS              (400U)           /**< センサーの安定待ち[ms] */
#define CPU0_SENSOR_PIVOT_MIN_MS           (600U)           /**< センサーピボットの最小[ms] */
#define CPU0_SENSOR_PIVOT_MAX_MS           (5000U)          /**< センサーピボットの最大[ms] */
#define CPU0_SENSOR_PIVOT_MIN_YAW_MDEG     (45000)          /**< センサーピボット最小のヨー[mdeg] */
#define CPU0_SENSOR_PIVOT_PROGRESS_MDEG    (5000)           /**< センサーピボットの進行[mdeg] */
#define CPU0_SENSOR_PIVOT_PROGRESS_MS      (2000U)          /**< センサーピボットの進行[ms] */
#define CPU0_SENSOR_COMMIT_YAW_MDEG        (30000)          /**< 強い回避操舵を維持するヨー[mdeg] */
#define CPU0_SENSOR_COMMIT_STEERING_DEG    (28)             /**< 回避開始時の操舵下限[deg] */
#define CPU0_SENSOR_COMMIT_HOLD_STEERING_DEG (12)           /**< 回頭後に回避側を保つ操舵下限[deg] */
#define CPU0_SENSOR_COMMIT_MAX_MS          (3000U)          /**< 回避側を強制保持する最大[ms] */
#define CPU0_SENSOR_AVOID_RELISTEN_MS      (2500U)          /**< 通路通過後に停止・再聴取する最大回避時間[ms] */
#define CPU0_SENSOR_ESCAPE_DIRECTION_SIDE_DELTA_MM (150U)   /**< 脱出方向を空き側優先にする左右差[mm] */
#define CPU0_SENSOR_ESCAPE_TARGET_DEADBAND_DEG (10)         /**< 脱出方向に音源方位を使う最小角度[deg] */
#define CPU0_SENSOR_BACKUP_MS              (700U)           /**< 正面近接後の短距離後退時間[ms] */
#define CPU0_SENSOR_BACKUP_RPM             (85)             /**< 正面近接後の後退RPM絶対値 */
#define CPU0_SENSOR_MAX_RECOVERY_ATTEMPTS  (2U)             /**< 後退・再旋回の安全な最大回数 */
#define CPU0_SENSOR_MAX_STEP_MS            (500U)           /**< センサー最大の更新幅[ms] */
/* 車体中心の鉛直Z軸を使う。現行配線の実機ログでは、左旋回指令が正、右旋回指令が負となる。 */
#define CPU0_SENSOR_YAW_AXIS               (2U)             /**< 車体ヨーに使う鉛直Z軸 */
#define CPU0_SENSOR_YAW_RIGHT_SIGN         (-1)             /**< センサーヨー右の符号 */
#define CPU0_SENSOR_YAW_DEADBAND_DPS_X10   (30)             /**< センサーヨーの不感帯[0.1dps] */
/*
 * 駆動系の実機校正値（始動境界 155‰、確実動作 200‰）に基づくRPM設定:
 * 実効Duty = RPM * (1000 / 300) * (800 / 1000) = RPM * 2.667‰
 * - 85 RPM  => 実効 227‰ (最低下限ガード: 200‰デッドバンドを確実に突破)
 * - 100 RPM => 実効 267‰ (注意減速時 / 最小並進旋回)
 * - 120 RPM => 実効 320‰ (通常前進)
 * - 150 RPM => 実効 400‰ (最小並進旋回の始動余裕)
 */
#define CPU0_SENSOR_FORWARD_RPM            (120)            /**< センサーの前進[RPM] */
#define CPU0_SENSOR_CAUTION_RPM            (100)            /**< センサーの注意[RPM] */
#define CPU0_SENSOR_MIN_FORWARD_RPM        (85)             /**< センサー最小の前進[RPM] */
#define CPU0_SENSOR_ESCAPE_TURN_RPM        (100)            /**< 最小並進旋回の左右RPM絶対値 */
#define CPU0_SENSOR_STEERING_MIN_DEG       (1)              /**< センサー操舵の最小[deg] */
#define CPU0_SENSOR_STEERING_MAX_DEG       (45)             /**< センサー操舵の最大[deg] */
#define CPU0_SENSOR_IMU_MAX_TILT_MG        (700)            /**< センサーIMU最大傾きのMG */
#define CPU0_SENSOR_IMU_MAX_SHOCK_L1_MG    (2400)           /**< センサーIMU最大衝撃L1のMG */
#define CPU0_SENSOR_IMU_MAX_GYRO_DPS_X10   (2000)           /**< センサーIMU最大の角速度[0.1dps] */

#define CPU0_SOUND_DOA_ZERO_OFFSET_DEG     (0)              /**< 音響DoAゼロの補正[deg] */
/* 実機走行ログで、右前方の音がXVFの小さい負角（例: 326°）として報告された。
 * 車体座標の「右正」へ合わせるため、XVFの時計回り値を反転する。 */
#define CPU0_SOUND_DOA_CLOCKWISE_POSITIVE  (0U)             /**< 音響DoA時計回りの正方向 */
#define CPU0_SOUND_TRACK_MIN_CONFIDENCE    (40U)            /**< 走行に採用する循環DoA品質下限[0..100] */
/* 平滑化DoAが生DoAから大きく遅れた観測は、旋回中の前回方位を示すことがある。
 * 静止聴取・追従判定では、両者がこの差分内で一致する観測だけを走行に採用する。 */
#define CPU0_SOUND_RAW_FILTER_MAX_DELTA_DEG (20)            /**< 生DoAと平滑DoAの許容差[deg] */
#define CPU0_SOUND_TRIGGER_DBFS_X100       (-4500)          /**< 音響の開始[0.01dBFS] */
#define CPU0_SOUND_RELEASE_DBFS_X100       (-4800)          /**< 音響の解除[0.01dBFS] */
#define CPU0_SOUND_DOA_SETTLE_MS           (500U)           /**< 音響DoAの安定待ち[ms] */
#define CPU0_SOUND_DOA_ACQUIRE_TIMEOUT_MS  (2000U)          /**< 音響DoA取得の期限[ms] */
#define CPU0_SOUND_DOA_SAMPLE_COUNT        (5U)             /**< 音響DoAサンプルの個数 */
#define CPU0_SOUND_DOA_STABLE_WIDTH_DEG    (20)             /**< 音響DoA安定の幅[deg] */
#define CPU0_SOUND_FRONT_TOLERANCE_DEG     (0)              /**< 音響正面の許容幅[deg] */
#define CPU0_SOUND_STEERING_MIN_DEG        (1)              /**< 音響操舵の最小[deg] */
#define CPU0_SOUND_STEERING_MAX_DEG        (45)             /**< 音響操舵の最大[deg] */
#define CPU0_SOUND_SPIN_THRESHOLD_DEG      (120)            /**< 後方音源の最小並進回頭DoA[deg] */
#define CPU0_SOUND_AVOID_REORIENT_THRESHOLD_DEG (35)        /**< 回避後に音源向きへ静止旋回する最小DoA[deg] */
#define CPU0_SOUND_AVOID_REORIENT_MAX_ATTEMPTS (2U)         /**< 回避後に正面へ向き直す最大試行回数 */
#define CPU0_SOUND_SPIN_RPM                (100)            /**< 最小並進回頭の左右車輪目標RPM絶対値 */
#define CPU0_SOUND_SPIN_SLOW_RPM           (85)             /**< 最小並進回頭の終端減速RPM絶対値 */
#define CPU0_SOUND_SPIN_SERVO_DEG          (35)             /**< 最小並進回頭の各舵輪角度絶対値[deg] */
#define CPU0_SOUND_SPIN_FRONT_RESERVE_DEG  (15)             /**< 前進操舵へ渡す残角[deg] */
#define CPU0_SOUND_SPIN_MAX_YAW_MDEG       (180000)         /**< 最小並進回頭一回の最大目標ヨー[mdeg] (180度回頭を許容) */
#define CPU0_SOUND_SPIN_SLOWDOWN_MDEG      (20000)          /**< 終端減速を始める残ヨー[mdeg] */
#define CPU0_SOUND_SPIN_MAX_MS             (12000U)         /**< 最小並進回頭の安全上限時間[ms] (180度旋回完了まで粘る) */
#define CPU0_SOUND_SPIN_PROGRESS_MS        (2500U)          /**< 最小並進回頭の進行確認時間[ms] (始動探索の余裕を確保) */
#define CPU0_SOUND_SPIN_PROGRESS_MDEG      (3000)           /**< 進行成立とみなす最小ヨー角[mdeg] */
#define CPU0_SOUND_SPIN_FAILURE_HOLD_MS    (500U)           /**< 進行不足の通知保持[ms] */
/* 最小並進旋回中にもDoAを監視する。車体が意図と逆へ回って音源方位が
 * 累計20度以上遠ざかった場合は、配線・摩擦差を吸収するため一回だけ
 * 駆動方向を反転する。正面域へ入ったらIMU目標を待たずに旋回を終える。 */
#define CPU0_SOUND_SPIN_DOA_REVERSE_DEG    (20)             /**< DoA逆進行で駆動を反転する累積角度[deg] */
#define CPU0_SOUND_SPIN_DOA_FRONT_DEG      (20)             /**< DoAで旋回完了とする正面域[deg] */
/* スピンターン始動トルク探索＆定常回転維持（Ramp-to-Motion Breakaway Hold）設定 */
#define CPU0_SPIN_RAMP_START_RPM           (60)             /**< 始動探索の初期RPM (実効Duty 160‰) */
#define CPU0_SPIN_RAMP_STEP_RPM            (10)             /**< 100ms周期ごとのランプ増加量[RPM] */
#define CPU0_SPIN_RAMP_MAX_RPM             (220)            /**< 始動探索の上限RPM (ジャイロ変化まで許容する最大Duty 587‰) */
#define CPU0_SPIN_MOTION_DETECT_DPS_X10    (60)             /**< 車体回転開始検知の角速度閾値[0.1dps] (6.0 dps) */
#define CPU0_SPIN_HOLD_MIN_DPS_X10         (50)             /**< 定常回転維持の下限角速度[0.1dps] (5.0 dps) */
#define CPU0_SPIN_HOLD_MAX_DPS_X10         (250)            /**< 定常回転維持の上限角速度[0.1dps] (25.0 dps) */
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

/* 音源追従の既定動作: 位置推定を直接の操舵へ使わず、対象音が続く間はDoAへ連続追従する。
 * 回避後だけは停止・再聴取し、対象音を失ったときは安全に停止する。 */
#define CPU0_SOUND_STOP_AND_LISTEN_ENABLE   (0U)             /**< ステップごとの停止聴取シーケンス有効化（0: 連続追従） */
#define CPU0_SOUND_USE_LOCALIZATION_FOR_STEERING (0U)       /**< 位置推定bearingを操舵へ使うか */
#define CPU0_SOUND_USE_LOCALIZATION_FOR_ARRIVAL (0U)        /**< 位置推定到着を停止理由へ使うか */
#define CPU0_SOUND_ALLOW_CONTROL_MLP        (1U)            /**< 音源追従中にMLP操舵を採用するか */
/* 現場学習音響識別の走行反映: 0は従来互換（DoA+音量追従）、 */
/* 1は学習見本一致時のみ追従する。 */
#define CPU0_SOUND_REQUIRE_IDENTIFIER_MATCH (1U)            /**< 音響必須識別の一致 */
#define CPU0_SOUND_IDENTIFIER_TIMEOUT_MS   (2500U)          /**< 音響識別の期限[ms] */

/* bearing-only音源位置推定。単一DoAでは距離を確定せず、移動基線と交差角を必須とする。 */
#define CPU0_SOUND_LOCALIZATION_MAX_OBSERVATIONS (12U)     /**< 位置推定へ保持する方位観測数 */
#define CPU0_SOUND_LOCALIZATION_MIN_CONFIDENCE (60U)       /**< 採用するDoA品質下限[0..100] */
#define CPU0_SOUND_LOCALIZATION_MIN_BASELINE_MM (150U)     /**< 推定に必要な最小移動基線[mm] */
#define CPU0_SOUND_LOCALIZATION_MIN_CROSSING_DEG (12U)     /**< 推定に必要な最小交差角[deg] */
#define CPU0_SOUND_LOCALIZATION_MAX_RESIDUAL_MM (250U)     /**< 方位線残差RMS上限[mm] */
#define CPU0_SOUND_LOCALIZATION_MAX_RANGE_MM (10000U)      /**< 採用する音源距離上限[mm] */
#define CPU0_SOUND_LOCALIZATION_MAX_JUMP_MM (1000U)        /**< 連続推定位置の最大変化[mm] */
#define CPU0_SOUND_LOCALIZATION_OBSERVATION_MAX_AGE_MS (5000U) /**< 方位観測保持時間[ms] */
#define CPU0_SOUND_LOCALIZATION_HOLD_MS     (3000U)         /**< 最終有効音から目標を保持する時間[ms] */
#define CPU0_SOUND_ARRIVAL_RANGE_MM         (450U)          /**< 到着候補の音源距離[mm] */
#define CPU0_SOUND_ARRIVAL_BEARING_DEG      (25U)           /**< 到着確認の正面方位幅[deg] */
#define CPU0_SOUND_ARRIVAL_MIN_CONFIDENCE   (65U)           /**< 到着確認の位置品質下限[0..100] */
#define CPU0_SOUND_ARRIVAL_STRONG_CONFIDENCE (80U)         /**< 正面外で許す強い位置品質[0..100] */
#define CPU0_SOUND_ARRIVAL_CONFIRM_COUNT    (3U)            /**< 到着確定に必要な有効観測数 */
#define CPU0_SOUND_ARRIVAL_VERIFY_MS        (250U)          /**< 到着候補を維持する時間[ms] */

/* TFLM完全int8障害物回避制御MLP（Policy Distillation）の有効化設定 */
#ifndef CPU0_USE_CONTROL_MLP
#define CPU0_USE_CONTROL_MLP               (1U)             /**< TFLM制御MLP障害物回避の有効化(0: ルールベース, 1: MLP) */
#endif

#endif /* SEROV_CPU0_CONFIG_CONTROL_H */
