/** =================================================================*
 * @file   task_infer.c
 * @brief  CPU0音響背景学習と現場見本照合
 * ================================================================= */
#include "task_infer.h"                                    /* 推論タスク公開API */

#include "config/task_config.h"                            /* 推論タスク優先度・スタック */
#include "services/background_model.h"                     /* 固定乱数AEとRLSデコーダ */
#include "task_acoustic_link.h"                            /* 完成特徴量パッチ取得 */
#include <string.h>                                         /* 結果・背景デコーダコピー */

#define CPU0_INFER_EVENT_FEATURE_READY     (1UL << 0)
#define CPU0_INFER_EVENT_MASK              ((UINT) CPU0_INFER_EVENT_FEATURE_READY)

LOCAL void task_infer_entry(INT stacd, void * exinf);
LOCAL void task_infer_resources_delete(void);
LOCAL void task_infer_result_clear(task_infer_result_t * p_result);
LOCAL void task_infer_feature_process(void);

LOCAL T_CMTX const infer_mutex_config = {
    .mtxatr = TA_INHERIT,
    .ceilpri = 0,
};
LOCAL T_CFLG const infer_event_config = {
    .flgatr = TA_TFIFO | TA_WSGL,
    .iflgptn = 0U,
};
LOCAL T_CTSK const infer_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = (FP) task_infer_entry,
    .itskpri = CPU0_INFER_TASK_PRIORITY,
    .stksz = CPU0_INFER_TASK_STACK_SIZE,
    .bufptr = NULL,
};

LOCAL ID infer_task_id;
LOCAL ID infer_mutex_id;
LOCAL ID infer_event_id;
LOCAL BOOL infer_task_started;
LOCAL BOOL infer_result_ready;
LOCAL BOOL infer_storage_valid;
LOCAL UB infer_target_peak_bin;
LOCAL float infer_bin_weights[CPU0_ACOUSTIC_FEATURE_BIN_COUNT];
LOCAL UW infer_last_feature_generation;
LOCAL prototype_storage_data_t infer_storage_data;
LOCAL background_model_state_t infer_background_model;
LOCAL acoustic_feature_patch_t infer_feature_patch;
LOCAL task_infer_result_t infer_result;

EXPORT volatile BOOL g_task_infer_available;
EXPORT volatile UW g_task_infer_feature_generation;
EXPORT volatile UW g_task_infer_inference_count;
EXPORT volatile UW g_task_infer_failure_count;
EXPORT volatile UW g_task_infer_match_count;
EXPORT volatile ER g_task_infer_last_kernel_error;

/** =================================================================*
 * @brief  公開結果を安全側の未判定状態へ初期化
 * ================================================================= */
LOCAL void task_infer_result_clear(task_infer_result_t * p_result) {
    memset(p_result, 0, sizeof(*p_result));
    p_result->identifier.sample_index = ~(UW) 0U;
    p_result->identifier.minimum_cosine_distance = -1.0F;
    p_result->identifier.threshold = -1.0F;
    p_result->identifier.status = CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_INVALID;
}

/** =================================================================*
 * @brief  任意起動した資源だけを解放する
 * ================================================================= */
LOCAL void task_infer_resources_delete(void) {
    if (infer_task_id > 0) {
        if (infer_task_started) {
            (void) tk_ter_tsk(infer_task_id);
        }
        (void) tk_del_tsk(infer_task_id);
        infer_task_id = 0;
        infer_task_started = FALSE;
    }
    if (infer_event_id > 0) {
        (void) tk_del_flg(infer_event_id);
        infer_event_id = 0;
    }
    if (infer_mutex_id > 0) {
        (void) tk_del_mtx(infer_mutex_id);
        infer_mutex_id = 0;
    }
}

/** =================================================================*
 * @brief  走行機能を止めない任意の音響タスク起動
 * ================================================================= */
EXPORT void task_infer_start_optional(void) {
    task_infer_resources_delete();
    infer_result_ready = FALSE;
    infer_storage_valid = FALSE;
    infer_target_peak_bin = 255U;
    infer_last_feature_generation = 0U;
    memset(&infer_storage_data, 0, sizeof(infer_storage_data));
    background_model_init(&infer_background_model, CPU0_BACKGROUND_MODEL_DEFAULT_SEED);
    task_infer_result_clear(&infer_result);
    g_task_infer_available = FALSE;
    g_task_infer_feature_generation = 0U;
    g_task_infer_inference_count = 0U;
    g_task_infer_failure_count = 0U;
    g_task_infer_match_count = 0U;
    g_task_infer_last_kernel_error = E_OK;

    infer_mutex_id = tk_cre_mtx(&infer_mutex_config);
    if (infer_mutex_id <= 0) {
        g_task_infer_last_kernel_error = (ER) infer_mutex_id;
        infer_mutex_id = 0;
        return;
    }
    infer_event_id = tk_cre_flg(&infer_event_config);
    if (infer_event_id <= 0) {
        g_task_infer_last_kernel_error = (ER) infer_event_id;
        infer_event_id = 0;
        task_infer_resources_delete();
        return;
    }
    infer_task_id = tk_cre_tsk(&infer_task_config);
    if (infer_task_id <= 0) {
        g_task_infer_last_kernel_error = (ER) infer_task_id;
        infer_task_id = 0;
        task_infer_resources_delete();
        return;
    }
    g_task_infer_last_kernel_error = tk_sta_tsk(infer_task_id, 0);
    if (E_OK != g_task_infer_last_kernel_error) {
        task_infer_resources_delete();
        return;
    }
    infer_task_started = TRUE;
    g_task_infer_available = TRUE;
}

EXPORT void task_infer_stop(void) {
    task_infer_resources_delete();
    g_task_infer_available = FALSE;
}

EXPORT ER task_infer_notify_feature_ready(void) {
    if (infer_event_id <= 0) {
        return E_NOEXS;
    }
    ER const err = tk_set_flg(infer_event_id, CPU0_INFER_EVENT_FEATURE_READY);
    g_task_infer_last_kernel_error = err;
    return err;
}

/** =================================================================*
 * @brief  保存済み背景デコーダと5見本をタスク側へ反映
 * @details Pは保存しないため、復元後にI/0.01から再開する。
 * ================================================================= */
EXPORT ER task_infer_prototype_set(const prototype_storage_data_t * p_data, BOOL storage_valid) {
    if (NULL == p_data) {
        return E_PAR;
    }
    if (infer_mutex_id <= 0) {
        return E_NOEXS;
    }
    ER const err = tk_loc_mtx(infer_mutex_id, TMO_POL);
    if (E_OK != err) {
        return err;
    }
    infer_storage_data = *p_data;
    infer_storage_valid = storage_valid;

    infer_target_peak_bin = 255U;
    for (UW b = 0U; b < CPU0_ACOUSTIC_FEATURE_BIN_COUNT; b++) {
        infer_bin_weights[b] = 1.0F;
    }
    if (storage_valid && (p_data->sample_count > 0U)) {
        if (p_data->target_peak_bin < CPU0_ACOUSTIC_FEATURE_BIN_COUNT) {
            infer_target_peak_bin = p_data->target_peak_bin;
        } else {
            infer_target_peak_bin = acoustic_identifier_find_peak_bin((const B *) p_data->samples,
                                                                      p_data->sample_count);
        }
        acoustic_identifier_build_weights(infer_target_peak_bin, infer_bin_weights);
    }

    background_model_init(&infer_background_model,
                          (0U == p_data->encoder_seed) ? CPU0_BACKGROUND_MODEL_DEFAULT_SEED : p_data->encoder_seed);
    if (storage_valid) {
        memcpy(infer_background_model.decoder, p_data->background_decoder,
               sizeof(infer_background_model.decoder));
    }
    ER const unlock_err = tk_unl_mtx(infer_mutex_id);
    g_task_infer_last_kernel_error = (E_OK == unlock_err) ? E_OK : unlock_err;
    return unlock_err;
}

/** =================================================================*
 * @brief  現在の背景デコーダとしきい値をMRAM保存用データへ取り出す
 * ================================================================= */
EXPORT ER task_infer_background_export(prototype_storage_data_t * p_data) {
    if (NULL == p_data) {
        return E_PAR;
    }
    if (infer_mutex_id <= 0) {
        return E_NOEXS;
    }
    ER const err = tk_loc_mtx(infer_mutex_id, TMO_FEVR);
    if (E_OK != err) {
        return err;
    }
    p_data->encoder_seed = infer_background_model.encoder_seed;
    memcpy(p_data->background_decoder, infer_background_model.decoder, sizeof(p_data->background_decoder));
    float threshold = 0.0F;
    if (background_model_mse_threshold(&infer_background_model, &threshold)) {
        p_data->background_mse_threshold = threshold;
    } else if (!infer_storage_valid) {
        p_data->background_mse_threshold = 0.0F;
    }
    return tk_unl_mtx(infer_mutex_id);
}

EXPORT ER task_infer_result_get(task_infer_result_t * p_result) {
    if (NULL == p_result) {
        return E_PAR;
    }
    if (infer_mutex_id <= 0) {
        return E_NOEXS;
    }
    ER const err = tk_loc_mtx(infer_mutex_id, TMO_POL);
    if (E_OK != err) {
        return err;
    }
    if (!infer_result_ready) {
        (void) tk_unl_mtx(infer_mutex_id);
        return E_NOEXS;
    }
    *p_result = infer_result;
    return tk_unl_mtx(infer_mutex_id);
}

EXPORT ER task_infer_prototype_get(prototype_storage_data_t * p_data, BOOL * p_storage_valid) {
    if ((NULL == p_data) || (NULL == p_storage_valid)) {
        return E_PAR;
    }
    if (infer_mutex_id <= 0) {
        return E_NOEXS;
    }
    ER const err = tk_loc_mtx(infer_mutex_id, TMO_POL);
    if (E_OK != err) {
        return err;
    }
    *p_data = infer_storage_data;
    *p_storage_valid = infer_storage_valid;
    return tk_unl_mtx(infer_mutex_id);
}

EXPORT ER task_infer_prototype_telemetry_get(task_infer_prototype_telemetry_t * p_telemetry) {
    if (NULL == p_telemetry) {
        return E_PAR;
    }
    if (infer_mutex_id <= 0) {
        return E_NOEXS;
    }
    ER const err = tk_loc_mtx(infer_mutex_id, TMO_POL);
    if (E_OK != err) {
        return err;
    }
    p_telemetry->storage_valid = infer_storage_valid;
    p_telemetry->sample_count = infer_storage_data.sample_count;
    p_telemetry->target_peak_bin = infer_target_peak_bin;
    p_telemetry->identifier_threshold = infer_storage_data.identifier_threshold;
    return tk_unl_mtx(infer_mutex_id);
}

/** =================================================================*
 * @brief  完成特徴量を背景学習・能動要約・個別見本照合へ渡す
 * @details 音響リンクmutexはfeature_getのコピーだけで解放され、以降のRLSとcosine計算は
 *          推論タスク自身のmutexだけで実行する。
 * ================================================================= */
LOCAL void task_infer_feature_process(void) {
    UW generation = 0U;
    ER const feature_err = task_acoustic_link_feature_get(&infer_feature_patch, &generation);
    if ((E_OK != feature_err) || (generation == infer_last_feature_generation)) {
        return;
    }
    infer_last_feature_generation = generation;
    g_task_infer_feature_generation = generation;
    g_task_infer_inference_count++;

    ER const lock_err = tk_loc_mtx(infer_mutex_id, TMO_FEVR);
    if (E_OK != lock_err) {
        g_task_infer_last_kernel_error = lock_err;
        g_task_infer_failure_count++;
        return;
    }

    static task_infer_result_t next;
    task_infer_result_clear(&next);
    next.feature_generation = generation;
    BOOL active_mse_found = FALSE;
    for (UW frame = 0U; frame < CPU0_ACOUSTIC_EVENT_FRAME_COUNT; frame++) {
        const B * const p_frame = &infer_feature_patch.frames[frame][0];
        BOOL const active = acoustic_identifier_frame_is_active(p_frame);
        float mse = 0.0F;
        if (!background_model_observe(&infer_background_model, p_frame, active, &mse)) {
            g_task_infer_failure_count++;
            continue;
        }
        if (active && ((!active_mse_found) || (mse > next.event_mse))) {
            next.event_mse = mse;
            active_mse_found = TRUE;
        }
    }
    next.summary_valid = acoustic_identifier_summary_create((const B *) infer_feature_patch.frames,
                                                             CPU0_ACOUSTIC_EVENT_FRAME_COUNT,
                                                             next.summary, &next.active_frame_count);
    next.background_threshold_valid = background_model_mse_threshold(&infer_background_model,
                                                                       &next.background_threshold);
    if ((!next.background_threshold_valid) && infer_storage_valid) {
        next.background_threshold = infer_storage_data.background_mse_threshold;
        next.background_threshold_valid = TRUE;
    }
    next.background_anomaly = active_mse_found && next.background_threshold_valid &&
                              (next.event_mse > next.background_threshold);
    acoustic_identifier_summary_classify(next.summary, next.summary_valid, next.active_frame_count,
                                         (const B *) infer_storage_data.samples,
                                         infer_storage_data.sample_count,
                                         infer_bin_weights,
                                         infer_storage_data.identifier_threshold, &next.identifier);
    if (CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_TARGET == next.identifier.status) {
        g_task_infer_match_count++;
    }
    infer_result = next;
    infer_result_ready = TRUE;
    (void) tk_unl_mtx(infer_mutex_id);
}

LOCAL void task_infer_entry(INT stacd, void * exinf) {
    (void) stacd;
    (void) exinf;
    while (1) {
        UINT pattern = 0U;
        ER const err = tk_wai_flg(infer_event_id, CPU0_INFER_EVENT_MASK,
                                  TWF_ORW | TWF_BITCLR, &pattern, TMO_FEVR);
        g_task_infer_last_kernel_error = err;
        if ((E_OK == err) && (0U != (pattern & CPU0_INFER_EVENT_FEATURE_READY))) {
            task_infer_feature_process();
        }
    }
}
