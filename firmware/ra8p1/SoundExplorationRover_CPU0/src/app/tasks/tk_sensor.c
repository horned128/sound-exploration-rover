/** =================================================================*
 * @file   tk_sensor.c
 * @brief  CPU0センサー取得タスク実装
 * ================================================================= */
#include "tk_sensor.h"                                     /* センサー取得タスクAPI */
#include "../../cpu0_config.h"                              /* センサー周期、優先度 */

LOCAL void cpu0_sensor_task(INT stacd, void * exinf);      /* センサー取得タスク本体 */
LOCAL void cpu0_sensor_snapshot_publish(const cpu0_sensor_snapshot_t * p_snapshot); /* 状態反映 */
LOCAL void cpu0_sensor_snapshot_mark_unavailable(fsp_err_t error); /* 初期化失敗反映 */

/**< センサースナップショットを保護するμT-Kernel mutex設定 */
LOCAL T_CMTX const sensor_mutex_config = {
    .mtxatr = TA_INHERIT,
    .ceilpri = 0,
};

/**< I2Cセンサーを定周期取得するタスク設定 */
LOCAL T_CTSK const sensor_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = (FP) cpu0_sensor_task,
    .itskpri = CPU0_SENSOR_TASK_PRIORITY,
    .stksz = CPU0_SENSOR_TASK_STACK_SIZE,
    .bufptr = NULL,
};

LOCAL ID sensor_task_id;                                    /**< センサータスクID */
LOCAL ID sensor_mutex_id;                                   /**< スナップショットmutex ID */
LOCAL BOOL sensor_task_started;                             /**< センサータスク開始状態 */
LOCAL cpu0_sensor_snapshot_t sensor_snapshot;               /**< mutexで保護する最新値 */

EXPORT volatile UH g_cpu0_sensor_tof_distance_mm[CPU0_SENSOR_TOF_COUNT]; /**< ToF距離 */
EXPORT volatile H g_cpu0_sensor_accel_mg[3];                 /**< IMU加速度 */
EXPORT volatile H g_cpu0_sensor_gyro_dps_x10[3];             /**< IMU角速度 */
EXPORT volatile UW g_cpu0_sensor_age_ms;                     /**< センサー値経過時間 */
EXPORT volatile UW g_cpu0_sensor_update_count;               /**< 正常更新回数 */
EXPORT volatile UW g_cpu0_sensor_error_flags;                /**< 最終センサーerror bit */
EXPORT volatile W g_cpu0_sensor_last_error;                  /**< 最終FSPエラー */
EXPORT volatile UB g_cpu0_sensor_valid_flags;                /**< ToF/IMU valid bit */
EXPORT volatile BOOL g_cpu0_sensor_initialized;              /**< 全センサー初期化状態 */

/** =================================================================*
 * @brief  最新スナップショットをmutex下で公開しLive Watch値も更新
 * @param[in] p_snapshot 公開する最新値
 * ================================================================= */
LOCAL void cpu0_sensor_snapshot_publish(const cpu0_sensor_snapshot_t * p_snapshot) {
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
        g_cpu0_sensor_tof_distance_mm[index] = p_snapshot->tof_distance_mm[index];
    }
    for (UW axis = 0U; axis < 3U; axis++) {
        g_cpu0_sensor_accel_mg[axis] = p_snapshot->accel_mg[axis];
        g_cpu0_sensor_gyro_dps_x10[axis] = p_snapshot->gyro_dps_x10[axis];
    }
    g_cpu0_sensor_age_ms = p_snapshot->age_ms;
    g_cpu0_sensor_update_count = p_snapshot->update_count;
    g_cpu0_sensor_error_flags = p_snapshot->error_flags;
    g_cpu0_sensor_last_error = p_snapshot->last_error;
    g_cpu0_sensor_valid_flags = p_snapshot->valid_flags;
    g_cpu0_sensor_initialized = p_snapshot->initialized;
}

/** =================================================================*
 * @brief  センサー初期化失敗を安全停止用スナップショットへ反映
 * @param[in] error 最終FSPエラー
 * ================================================================= */
LOCAL void cpu0_sensor_snapshot_mark_unavailable(fsp_err_t error) {
    if (sensor_snapshot.age_ms <= UINT32_MAX - CPU0_SENSOR_RETRY_PERIOD_MS) {
        sensor_snapshot.age_ms += CPU0_SENSOR_RETRY_PERIOD_MS;
    } else {
        sensor_snapshot.age_ms = UINT32_MAX;
    }
    sensor_snapshot.initialized = FALSE;
    sensor_snapshot.valid_flags = 0U;
    sensor_snapshot.error_flags = CPU0_SENSOR_ERROR_I2C_INIT;
    sensor_snapshot.last_error = (W) error;
    cpu0_sensor_snapshot_publish(&sensor_snapshot);
}

/** =================================================================*
 * @brief  センサータスクと共有資源を生成
 * @return CPU0異常コード
 * ================================================================= */
EXPORT cpu0_fault_t cpu0_sensor_task_create(void) {
    sensor_task_id = 0;
    sensor_mutex_id = 0;
    sensor_task_started = FALSE;
    sensor_snapshot = (cpu0_sensor_snapshot_t){
        .age_ms = UINT32_MAX,
        .last_error = (W) FSP_ERR_NOT_OPEN,
        .error_flags = CPU0_SENSOR_ERROR_I2C_INIT,
        .initialized = FALSE,
    };
    cpu0_sensor_snapshot_publish(&sensor_snapshot);

    sensor_mutex_id = tk_cre_mtx(&sensor_mutex_config);
    if (sensor_mutex_id <= 0) {
        sensor_mutex_id = 0;
        return CPU0_FAULT_TASK_CREATE;
    }

    sensor_task_id = tk_cre_tsk(&sensor_task_config);
    if (sensor_task_id <= 0) {
        sensor_task_id = 0;
        cpu0_sensor_task_delete();
        return CPU0_FAULT_TASK_CREATE;
    }

    return CPU0_FAULT_NONE;
}

/** =================================================================*
 * @brief  センサータスクを開始
 * @return CPU0異常コード
 * ================================================================= */
EXPORT cpu0_fault_t cpu0_sensor_task_start(void) {
    if (sensor_task_id <= 0) {
        return CPU0_FAULT_TASK_CREATE;
    }

    ER const err = tk_sta_tsk(sensor_task_id, 0);
    if (E_OK != err) {
        return CPU0_FAULT_TASK_START;
    }
    sensor_task_started = TRUE;
    return CPU0_FAULT_NONE;
}

/** =================================================================*
 * @brief  センサータスクと共有資源を解放
 * ================================================================= */
EXPORT void cpu0_sensor_task_delete(void) {
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
 * ================================================================= */
EXPORT ER cpu0_sensor_snapshot_get(cpu0_sensor_snapshot_t * p_snapshot) {
    if (NULL == p_snapshot) {
        return E_PAR;
    }
    if (sensor_mutex_id <= 0) {
        return E_NOEXS;
    }

    ER const err = tk_loc_mtx(sensor_mutex_id, TMO_POL);
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
LOCAL void cpu0_sensor_task(INT stacd, void * exinf) {
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
                cpu0_sensor_snapshot_mark_unavailable(init_err);
                (void) tk_dly_tsk(CPU0_SENSOR_RETRY_PERIOD_MS);
                continue;
            }
        }

        cpu0_sensor_snapshot_t next = {0};
        fsp_err_t const poll_err = sensor_hub_poll(&next);
        next.update_count = update_count;
        if (FSP_SUCCESS == poll_err) {
            update_count++;
            next.update_count = update_count;
        } else {
            next.valid_flags = 0U;
            next.error_flags |= CPU0_SENSOR_ERROR_STALE;
            next.last_error = (W) poll_err;
            sensor_hub_deinit();
            hub_ready = FALSE;
        }
        cpu0_sensor_snapshot_publish(&next);
        (void) tk_dly_tsk(CPU0_SENSOR_PERIOD_MS);
    }
}
