/** =================================================================*
 * @file   task_sensor.c
 * @brief  CPU0センサー取得タスク実装
 * ================================================================= */
#include "task_sensor.h"                                     /* センサー取得タスクAPI */
#include "config/sensor_config.h"                          /* センサー周期、再試行 */
#include "config/task_config.h"                            /* センサータスク優先度 */

LOCAL void task_sensor_entry(INT stacd, void * exinf);      /* センサー取得タスク本体 */
LOCAL void task_sensor_snapshot_publish(const sensor_snapshot_t * p_snapshot); /* 状態反映 */
LOCAL void task_sensor_snapshot_mark_unavailable(fsp_err_t error); /* 初期化失敗反映 */
LOCAL void task_sensor_snapshot_mark_stale(const sensor_snapshot_t * p_failure); /* 通信障害反映 */

/**< センサースナップショットを保護するμT-Kernel mutex設定 */
LOCAL T_CMTX const sensor_mutex_config = {
    .mtxatr = TA_INHERIT,
    .ceilpri = 0,
};

/**< I2Cセンサーを定周期取得するタスク設定 */
LOCAL T_CTSK const sensor_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = (FP) task_sensor_entry,
    .itskpri = CPU0_SENSOR_TASK_PRIORITY,
    .stksz = CPU0_SENSOR_TASK_STACK_SIZE,
    .bufptr = NULL,
};

LOCAL ID sensor_task_id;                                    /**< センサータスクID */
LOCAL ID sensor_mutex_id;                                   /**< スナップショットmutex ID */
LOCAL BOOL sensor_task_started;                             /**< センサータスク開始状態 */
LOCAL sensor_snapshot_t sensor_snapshot;               /**< mutexで保護する最新値 */

EXPORT volatile UH g_task_sensor_tof_distance_mm[CPU0_SENSOR_TOF_COUNT]; /**< ToF距離 */
EXPORT volatile H g_task_sensor_accel_mg[3];                 /**< IMU加速度 */
EXPORT volatile H g_task_sensor_gyro_dps_x10[3];             /**< IMU角速度 */
EXPORT volatile UW g_task_sensor_age_ms;                     /**< センサー値経過時間 */
EXPORT volatile UW g_task_sensor_update_count;               /**< 正常更新回数 */
EXPORT volatile UW g_task_sensor_error_flags;                /**< 現在のセンサー異常ビット */
EXPORT volatile W g_task_sensor_last_error;                  /**< 現在のFSPエラー */
EXPORT volatile UB g_task_sensor_valid_flags;                /**< ToF/IMU有効ビット */
EXPORT volatile BOOL g_task_sensor_initialized;              /**< 全センサー初期化状態 */
EXPORT volatile UB g_task_sensor_tof_range_status[CPU0_SENSOR_TOF_COUNT]; /**< ToF生Range Status */
EXPORT volatile UB g_task_sensor_tof_result[CPU0_SENSOR_TOF_COUNT]; /**< ToF読出し結果分類 */
EXPORT volatile UB g_task_sensor_failure_kind;               /**< 現在の失敗分類 */
EXPORT volatile UB g_task_sensor_failure_device;             /**< 現在の失敗デバイス */
EXPORT volatile UB g_task_sensor_failure_stage;              /**< 現在の失敗段階 */
EXPORT volatile B g_task_sensor_failure_channel;             /**< 現在の失敗TCAチャネル */
EXPORT volatile UW g_task_sensor_i2c_transfer_timeout_count; /**< I2C転送タイムアウト累積回数 */
EXPORT volatile UW g_task_sensor_tof_data_ready_timeout_count[CPU0_SENSOR_TOF_COUNT]; /**< ToFデータ準備タイムアウト回数 */
EXPORT volatile UW g_task_sensor_invalid_data_count[CPU0_SENSOR_TOF_COUNT]; /**< ToF測距無効回数 */
EXPORT volatile UW g_task_sensor_hub_recovery_count;         /**< I2C障害後ハブ再初期化回数 */

/** =================================================================*
 * @brief  最新スナップショットをmutex下で公開しLive Watch値も更新
 * @param[in] p_snapshot 公開する最新値
 * ================================================================= */
LOCAL void task_sensor_snapshot_publish(const sensor_snapshot_t * p_snapshot) {
    if (NULL == p_snapshot) {
        return;
    }

    if (sensor_mutex_id > 0) {
        if (E_OK == tk_loc_mtx(sensor_mutex_id, TMO_FEVR)) {
            sensor_snapshot = *p_snapshot;
            (void) tk_unl_mtx(sensor_mutex_id);
        }
    }

    for (UW index = 0U; index < CPU0_SENSOR_TOF_COUNT; index++) {
        g_task_sensor_tof_distance_mm[index] = p_snapshot->tof_distance_mm[index];
    }
    for (UW axis = 0U; axis < 3U; axis++) {
        g_task_sensor_accel_mg[axis] = p_snapshot->accel_mg[axis];
        g_task_sensor_gyro_dps_x10[axis] = p_snapshot->gyro_dps_x10[axis];
    }
    g_task_sensor_age_ms = p_snapshot->age_ms;
    g_task_sensor_update_count = p_snapshot->update_count;
    g_task_sensor_error_flags = p_snapshot->error_flags;
    g_task_sensor_last_error = p_snapshot->last_error;
    g_task_sensor_valid_flags = p_snapshot->valid_flags;
    g_task_sensor_initialized = p_snapshot->initialized;
    for (UW index = 0U; index < CPU0_SENSOR_TOF_COUNT; index++) {
        g_task_sensor_tof_range_status[index] = p_snapshot->diagnostics.tof_range_status[index];
        g_task_sensor_tof_result[index] = p_snapshot->diagnostics.tof_result[index];
        if ((SENSOR_TOF_RESULT_RANGE_STATUS_INVALID == p_snapshot->diagnostics.tof_result[index]) ||
            (SENSOR_TOF_RESULT_DISTANCE_INVALID == p_snapshot->diagnostics.tof_result[index])) {
            g_task_sensor_invalid_data_count[index]++;
        }
        if (SENSOR_TOF_RESULT_DATA_READY_TIMEOUT == p_snapshot->diagnostics.tof_result[index]) {
            g_task_sensor_tof_data_ready_timeout_count[index]++;
        }
    }
    if (CPU0_SENSOR_FAILURE_I2C_TRANSFER_TIMEOUT == p_snapshot->diagnostics.failure_kind) {
        g_task_sensor_i2c_transfer_timeout_count++;
    }
    /*
     * 診断値は直近に公開したスナップショットの状態を常に表す。
     * 失敗時だけ更新すると、次の成功周期後もLive Watchに過去の失敗が
     * 残り続け、現在の状態と区別できなくなる。
     */
    g_task_sensor_failure_kind = (UB) p_snapshot->diagnostics.failure_kind;
    g_task_sensor_failure_device = (UB) p_snapshot->diagnostics.failure_device;
    g_task_sensor_failure_stage = (UB) p_snapshot->diagnostics.failure_stage;
    g_task_sensor_failure_channel = p_snapshot->diagnostics.failure_channel;
}

/** =================================================================*
 * @brief  センサー初期化失敗を安全停止用スナップショットへ反映
 * @param[in] error 最終FSPエラー
 * ================================================================= */
LOCAL void task_sensor_snapshot_mark_unavailable(fsp_err_t error) {
    if (sensor_snapshot.age_ms <= UINT32_MAX - CPU0_SENSOR_RETRY_PERIOD_MS) {
        sensor_snapshot.age_ms += CPU0_SENSOR_RETRY_PERIOD_MS;
    } else {
        sensor_snapshot.age_ms = UINT32_MAX;
    }
    sensor_snapshot.initialized = FALSE;
    sensor_snapshot.valid_flags = 0U;
    sensor_snapshot.error_flags = CPU0_SENSOR_ERROR_I2C_INIT;
    sensor_snapshot.last_error = (W) error;
    sensor_hub_diagnostics_get(&sensor_snapshot.diagnostics);
    task_sensor_snapshot_publish(&sensor_snapshot);
}

/** =================================================================*
 * @brief  通信障害を期限切れスナップショットへ反映
 * @details 最新値をvalidとして使い回さず、更新失敗と経過時間を安全側へ公開する。
 * @param[in] p_failure I2C転送障害を含む取得結果
 * ================================================================= */
LOCAL void task_sensor_snapshot_mark_stale(const sensor_snapshot_t * p_failure) {
    if (NULL == p_failure) {
        return;
    }

    if (sensor_snapshot.age_ms <= UINT32_MAX - CPU0_SENSOR_PERIOD_MS) {
        sensor_snapshot.age_ms += CPU0_SENSOR_PERIOD_MS;
    } else {
        sensor_snapshot.age_ms = UINT32_MAX;
    }
    sensor_snapshot.update_count = p_failure->update_count;
    sensor_snapshot.initialized = FALSE;
    sensor_snapshot.valid_flags = 0U;
    sensor_snapshot.error_flags = p_failure->error_flags | CPU0_SENSOR_ERROR_STALE;
    sensor_snapshot.last_error = p_failure->last_error;
    sensor_snapshot.diagnostics = p_failure->diagnostics;
    task_sensor_snapshot_publish(&sensor_snapshot);
}

/** =================================================================*
 * @brief  センサータスクと共有資源を生成
 * @return CPU0異常コード
 * ================================================================= */
EXPORT app_fault_t task_sensor_create(void) {
    sensor_task_id = 0;
    sensor_mutex_id = 0;
    sensor_task_started = FALSE;
    sensor_snapshot = (sensor_snapshot_t){
        .age_ms = UINT32_MAX,
        .last_error = (W) FSP_ERR_NOT_OPEN,
        .error_flags = CPU0_SENSOR_ERROR_I2C_INIT,
        .initialized = FALSE,
        .diagnostics = {
            .failure_kind = CPU0_SENSOR_FAILURE_TRANSPORT,
            .failure_device = CPU0_SENSOR_FAILURE_DEVICE_I2C_BUS,
            .failure_stage = CPU0_SENSOR_FAILURE_STAGE_I2C_OPEN,
            .failure_channel = CPU0_SENSOR_FAILURE_CHANNEL_NONE,
        },
    };
    for (UW index = 0U; index < CPU0_SENSOR_TOF_COUNT; index++) {
        sensor_snapshot.diagnostics.tof_range_status[index] = SENSOR_TOF_RANGE_STATUS_UNAVAILABLE;
        sensor_snapshot.diagnostics.tof_result[index] = (UB) SENSOR_TOF_RESULT_TRANSPORT_ERROR;
    }
    g_task_sensor_i2c_transfer_timeout_count = 0U;
    g_task_sensor_hub_recovery_count = 0U;
    for (UW index = 0U; index < CPU0_SENSOR_TOF_COUNT; index++) {
        g_task_sensor_tof_data_ready_timeout_count[index] = 0U;
        g_task_sensor_invalid_data_count[index] = 0U;
    }
    task_sensor_snapshot_publish(&sensor_snapshot);

    sensor_mutex_id = tk_cre_mtx(&sensor_mutex_config);
    if (sensor_mutex_id <= 0) {
        sensor_mutex_id = 0;
        return APP_FAULT_TASK_CREATE;
    }

    sensor_task_id = tk_cre_tsk(&sensor_task_config);
    if (sensor_task_id <= 0) {
        sensor_task_id = 0;
        task_sensor_delete();
        return APP_FAULT_TASK_CREATE;
    }

    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  センサータスクを開始
 * @return CPU0異常コード
 * ================================================================= */
EXPORT app_fault_t task_sensor_start(void) {
    if (sensor_task_id <= 0) {
        return APP_FAULT_TASK_CREATE;
    }

    ER const err = tk_sta_tsk(sensor_task_id, 0);
    if (E_OK != err) {
        return APP_FAULT_TASK_START;
    }
    sensor_task_started = TRUE;
    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  センサータスクと共有資源を解放
 * ================================================================= */
EXPORT void task_sensor_delete(void) {
    if (sensor_task_id > 0) {
        if (sensor_task_started) {
            (void) tk_ter_tsk(sensor_task_id);
        }
        (void) tk_del_tsk(sensor_task_id);
        sensor_task_id = 0;
        sensor_task_started = FALSE;
    }
    sensor_hub_deinit();

    if (sensor_mutex_id > 0) {
        (void) tk_del_mtx(sensor_mutex_id);
        sensor_mutex_id = 0;
    }
}

/** =================================================================*
 * @brief  最新センサースナップショットを取得
 * @param[out] p_snapshot 取得先
 * @return μT-Kernelエラーコード
 * @details タスクコンテキスト専用。公開側がコピー中の場合だけ待機し、
 *          テレメトリに初期値を送らないようにする。
 * ================================================================= */
EXPORT ER task_sensor_snapshot_get(sensor_snapshot_t * p_snapshot) {
    if (NULL == p_snapshot) {
        return E_PAR;
    }
    if (sensor_mutex_id <= 0) {
        return E_NOEXS;
    }

    ER const err = tk_loc_mtx(sensor_mutex_id, TMO_FEVR);
    if (E_OK != err) {
        return err;
    }
    *p_snapshot = sensor_snapshot;
    return tk_unl_mtx(sensor_mutex_id);
}

/** =================================================================*
 * @brief  定周期センサー取得タスク
 * @param[in] stacd 起動コード（未使用）
 * @param[in] exinf 拡張情報（未使用）
 * ================================================================= */
LOCAL void task_sensor_entry(INT stacd, void * exinf) {
    (void) stacd;
    (void) exinf;

    BOOL hub_ready = FALSE;
    UW update_count = 0U;
    while (1) {
        if (!hub_ready) {
            fsp_err_t const init_err = sensor_hub_init();
            if (FSP_SUCCESS == init_err) {
                hub_ready = TRUE;
            } else {
                task_sensor_snapshot_mark_unavailable(init_err);
                (void) tk_dly_tsk(CPU0_SENSOR_RETRY_PERIOD_MS);
                continue;
            }
        }

        sensor_snapshot_t next = {0};
        fsp_err_t const poll_err = sensor_hub_poll(&next);
        next.update_count = update_count;
        if (FSP_SUCCESS == poll_err) {
            update_count++;
            next.update_count = update_count;
            task_sensor_snapshot_publish(&next);
        } else {
            g_task_sensor_hub_recovery_count++;
            sensor_hub_transport_fault_deinit();
            hub_ready = FALSE;
            task_sensor_snapshot_mark_stale(&next);
        }
        (void) tk_dly_tsk(CPU0_SENSOR_PERIOD_MS);
    }
}
