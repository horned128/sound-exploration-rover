/** =================================================================*
 * @file   task_infer.c
 * @brief  CPU0音響背景学習と現場見本照合
 * ================================================================= */
#include "task_infer.h"                                     /* 推論タスク公開API */
#include "config/task_config.h"                             /* 推論タスク優先度・スタック */
#include "services/background_model.h"                      /* 固定乱数AEとRLSデコーダ */
#include "task_acoustic_link.h"                             /* 完成特徴量パッチ取得 */
#if (CPU0_USE_ACOUSTIC_EMBEDDING_TFLM != 0U)
#include "ai/acoustic_embedding_identifier.h"              /* TFLM音響埋め込みCNN */
#endif
#include <string.h>                                         /* 結果・背景デコーダコピー */

#define CPU0_INFER_EVENT_FEATURE_READY     (1UL << 0)       /**< 完成特徴量の推論開始イベントビット */
#define CPU0_INFER_EVENT_MASK              /**< 推論タスクが待つイベントビット全体 */ \
    ((UINT) CPU0_INFER_EVENT_FEATURE_READY)

#if (CPU0_USE_ACOUSTIC_EMBEDDING_TFLM != 0U)
LOCAL float infer_current_embedding[ACOUSTIC_EMBEDDING_DIMENSION]; /**< 現在窓の64次元音響埋め込み */
/**< 現場学習で登録された5見本の64次元音響埋め込み */
LOCAL float infer_prototype_embeddings[CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT * ACOUSTIC_EMBEDDING_DIMENSION];
LOCAL BOOL infer_embedding_ready = FALSE;                   /**< 現場学習埋め込み配列の有効フラグ */
#endif

LOCAL void task_infer_entry(INT stacd, void * exinf);       /* 音響推論タスク本体 */
LOCAL void task_infer_resources_delete(void);               /* 音響推論資源解放 */
LOCAL void task_infer_result_clear(task_infer_result_t * p_result); /* 音響推論結果初期化 */
LOCAL void task_infer_feature_process(void);                /* 完成特徴量推論処理 */

/**< 音響推論結果を保護するmutex設定 */
LOCAL T_CMTX const infer_mutex_config = {
    .mtxatr = TA_INHERIT,
    .ceilpri = 0,
};
/**< 音響特徴量完成を通知するイベント設定 */
LOCAL T_CFLG const infer_event_config = {
    .flgatr = TA_TFIFO | TA_WSGL,
    .iflgptn = 0U,
};
/**< 音響推論タスクの生成設定 */
LOCAL T_CTSK const infer_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = (FP) task_infer_entry,
    .itskpri = CPU0_INFER_TASK_PRIORITY,
    .stksz = CPU0_INFER_TASK_STACK_SIZE,
    .bufptr = NULL,
};

LOCAL ID infer_task_id;                                     /**< 音響推論タスクID */
LOCAL ID infer_mutex_id;                                    /**< 音響推論mutex ID */
LOCAL ID infer_event_id;                                    /**< 音響推論イベントフラグID */
LOCAL BOOL infer_task_started;                              /**< 音響推論タスク起動状態 */
LOCAL BOOL infer_result_ready;                              /**< 音響推論結果の準備状態 */
LOCAL BOOL infer_storage_valid;                             /**< 現場見本保存データの有効状態 */
LOCAL UB infer_target_peak_bin;                             /**< 現場見本の代表ピークbin */
/**< 現場見本照合用bin重み */
LOCAL float infer_bin_weights[CPU0_ACOUSTIC_FEATURE_BIN_COUNT];
LOCAL UW infer_last_feature_generation;                     /**< 最後に処理した特徴量世代 */
LOCAL prototype_storage_data_t infer_storage_data;          /**< 推論タスクが保持する現場見本 */
LOCAL background_model_state_t infer_background_model;      /**< 推論タスクが保持する背景モデル */
LOCAL acoustic_feature_patch_t infer_feature_patch;         /**< 推論対象の完成特徴量 */
LOCAL task_infer_result_t infer_result;                     /**< 最新の音響推論結果 */

EXPORT volatile BOOL g_task_infer_available;                /**< 音響推論機能の利用可能状態 */
EXPORT volatile BOOL g_task_infer_tflm_available;           /**< TFLM音響モデルの利用可能状態 */
EXPORT volatile UW g_task_infer_feature_generation;         /**< 最後に処理した特徴量世代 */
EXPORT volatile UW g_task_infer_inference_count;            /**< 音響推論実行回数 */
EXPORT volatile UW g_task_infer_failure_count;              /**< 音響推論失敗回数 */
EXPORT volatile UW g_task_infer_match_count;                /**< 音響見本一致回数 */
EXPORT volatile UW g_task_infer_processing_last_ms;
EXPORT volatile UW g_task_infer_processing_max_ms;
EXPORT volatile ER g_task_infer_last_kernel_error;          /**< 音響推論タスクの最終Kernelエラー */

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
#if (CPU0_USE_ACOUSTIC_EMBEDDING_TFLM != 0U)
    g_task_infer_tflm_available = acoustic_embedding_identifier_init();
    infer_embedding_ready = FALSE;
    memset(infer_current_embedding, 0, sizeof(infer_current_embedding));
    memset(infer_prototype_embeddings, 0, sizeof(infer_prototype_embeddings));
#else
    g_task_infer_tflm_available = FALSE;
#endif
    task_infer_result_clear(&infer_result);
    g_task_infer_available = FALSE;
    g_task_infer_feature_generation = 0U;
    g_task_infer_inference_count = 0U;
    g_task_infer_failure_count = 0U;
    g_task_infer_match_count = 0U;
    g_task_infer_processing_last_ms = 0U;
    g_task_infer_processing_max_ms = 0U;
    g_task_infer_last_kernel_error = E_OK;

    /* 任意機能の資源は依存順に生成し、途中失敗時は同じ順序の逆順で解放する。 */
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

/** =================================================================*
 * @brief  任意音響推論タスク停止
 * ================================================================= */
EXPORT void task_infer_stop(void) {
    task_infer_resources_delete();
    g_task_infer_available = FALSE;
}

/** =================================================================*
 * @brief  音響特徴量完成イベント通知
 * @return μT-Kernelエラーコード
 * ================================================================= */
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
    ER const err = tk_loc_mtx(infer_mutex_id, TMO_FEVR);
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
        /* 旧MRAMに孤立見本が残っていても、実際に照合する4見本から重みを決める。 */
        infer_target_peak_bin = acoustic_identifier_consensus_peak_bin((const B *) p_data->samples,
                                                                        p_data->sample_count);
        acoustic_identifier_build_weights(infer_target_peak_bin, infer_bin_weights);
    }

    background_model_init(&infer_background_model,
                          (0U == p_data->encoder_seed) ? CPU0_BACKGROUND_MODEL_DEFAULT_SEED : p_data->encoder_seed);
    if (storage_valid) {
        memcpy(infer_background_model.decoder, p_data->background_decoder,
               sizeof(infer_background_model.decoder));
    }
#if (CPU0_USE_ACOUSTIC_EMBEDDING_TFLM != 0U)
    if (storage_valid && (p_data->sample_count > 0U) && (p_data->embedding_valid != 0U)) {
        memcpy(infer_prototype_embeddings, p_data->prototype_embeddings,
               sizeof(infer_prototype_embeddings));
        infer_embedding_ready = TRUE;
    } else {
        infer_embedding_ready = FALSE;
    }
#endif
    ER const unlock_err = tk_unl_mtx(infer_mutex_id);
    g_task_infer_last_kernel_error = (E_OK == unlock_err) ? E_OK : unlock_err;
    return unlock_err;
}

/** =================================================================*
 * @brief  現場見本音響埋め込み登録
 * @param[in] sample_index 登録先見本番号 (0〜4)
 * @return 処理結果 (E_OK: 成功, E_PAR: 引数不正, E_NOEXS: ミューテックス未生成)
 * ================================================================= */
EXPORT ER task_infer_prototype_embedding_register(UW sample_index) {
#if (CPU0_USE_ACOUSTIC_EMBEDDING_TFLM != 0U)
    if (sample_index >= CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT) {
        return E_PAR;
    }
    if (infer_mutex_id <= 0) {
        return E_NOEXS;
    }
    ER const err = tk_loc_mtx(infer_mutex_id, TMO_FEVR);
    if (E_OK != err) {
        return err;
    }
    float * p_dest = &infer_prototype_embeddings[sample_index * ACOUSTIC_EMBEDDING_DIMENSION];
    memcpy(p_dest, infer_current_embedding, sizeof(infer_current_embedding));
    if ((CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT - 1U) == sample_index) {
        infer_embedding_ready = TRUE;
    }
    return tk_unl_mtx(infer_mutex_id);
#else
    (void) sample_index;
    return E_OK;
#endif
}

/** =================================================================*
 * @brief  外れ値除外後の音響埋め込み配列同期圧縮
 * @param[in] mask 保持する見本のビットマスク
 * @return 処理結果 (E_OK: 成功, E_NOEXS: ミューテックス未生成)
 * ================================================================= */
EXPORT ER task_infer_prototype_embeddings_compact(UB mask) {
#if (CPU0_USE_ACOUSTIC_EMBEDDING_TFLM != 0U)
    if (infer_mutex_id <= 0) {
        return E_NOEXS;
    }
    ER const err = tk_loc_mtx(infer_mutex_id, TMO_FEVR);
    if (E_OK != err) {
        return err;
    }
    UW kept = 0U;
    for (UW index = 0U; index < CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT; index++) {
        if (mask & (1U << index)) {
            if (kept != index) {
                memcpy(&infer_prototype_embeddings[kept * ACOUSTIC_EMBEDDING_DIMENSION],
                       &infer_prototype_embeddings[index * ACOUSTIC_EMBEDDING_DIMENSION],
                       ACOUSTIC_EMBEDDING_DIMENSION * sizeof(float));
            }
            kept++;
        }
    }
    memset(&infer_prototype_embeddings[kept * ACOUSTIC_EMBEDDING_DIMENSION], 0,
           (CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT - kept) * ACOUSTIC_EMBEDDING_DIMENSION * sizeof(float));
    if (kept < CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT) {
        infer_embedding_ready = FALSE;
    }
    return tk_unl_mtx(infer_mutex_id);
#else
    (void) mask;
    return E_OK;
#endif
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

/** =================================================================*
 * @brief  最新の音響推論結果取得
 * @param[out] p_result 推論結果出力
 * @return μT-Kernelエラーコード
 * ================================================================= */
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

/** =================================================================*
 * @brief  保存済み現場見本取得
 * @param[out] p_data 保存データ出力
 * @param[out] p_storage_valid 保存データ有効フラグ
 * @return μT-Kernelエラーコード
 * ================================================================= */
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
#if (CPU0_USE_ACOUSTIC_EMBEDDING_TFLM != 0U)
    memcpy(p_data->prototype_embeddings, infer_prototype_embeddings,
           sizeof(p_data->prototype_embeddings));
    p_data->embedding_valid = (UB) (infer_embedding_ready ? 1U : 0U);
#else
    memset(p_data->prototype_embeddings, 0, sizeof(p_data->prototype_embeddings));
    p_data->embedding_valid = 0U;
#endif
    *p_storage_valid = infer_storage_valid;
    return tk_unl_mtx(infer_mutex_id);
}

/** =================================================================*
 * @brief  現場見本テレメトリ取得
 * @param[out] p_telemetry テレメトリ出力
 * @return μT-Kernelエラーコード
 * ================================================================= */
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
    /* 旧MRAMに広い値が保存されていても、画面には実際に使用する上限を出す。 */
    p_telemetry->identifier_threshold =
        (infer_storage_data.identifier_threshold > CPU0_ACOUSTIC_IDENTIFIER_THRESHOLD_MAX) ?
            CPU0_ACOUSTIC_IDENTIFIER_THRESHOLD_MAX : infer_storage_data.identifier_threshold;
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
    SYSTIM started = {0};
    (void) tk_get_otm(&started);

    /* タスクスタックを圧迫せず、mutex保護下で完成結果を公開する作業領域。 */
    LOCAL task_infer_result_t next;
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
    next.classifier_kind = CPU0_ACOUSTIC_CLASSIFIER_DSP_SUMMARY;
#if (CPU0_USE_ACOUSTIC_EMBEDDING_TFLM != 0U)
    /* TFLM音響埋め込みCNNによる高精度照合 (有効かつ登録済みかつパッチ有効時に適用) */
    if (next.summary_valid && g_task_infer_tflm_available &&
        acoustic_embedding_identifier_extract((const B *) infer_feature_patch.frames, infer_current_embedding)) {
        if (infer_storage_valid && (infer_storage_data.sample_count > 0U) && infer_embedding_ready) {
            acoustic_identifier_summary_output_t tflm_output;
            acoustic_embedding_identifier_classify(
                infer_current_embedding,
                infer_prototype_embeddings,
                infer_storage_data.sample_count,
                ACOUSTIC_EMBEDDING_SAFE_THRESHOLD,
                &tflm_output);
            if (CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_READY != tflm_output.status) {
                next.identifier = tflm_output;
                next.classifier_kind = CPU0_ACOUSTIC_CLASSIFIER_NN_EMBEDDING;
            }
        }
    }
#endif
    if (CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_TARGET == next.identifier.status) {
        g_task_infer_match_count++;
    }
    infer_result = next;
    infer_result_ready = TRUE;
    SYSTIM finished = {0};
    if (E_OK == tk_get_otm(&finished)) {
        g_task_infer_processing_last_ms = finished.lo - started.lo;
        if (g_task_infer_processing_last_ms > g_task_infer_processing_max_ms) {
            g_task_infer_processing_max_ms = g_task_infer_processing_last_ms;
        }
    }
    (void) tk_unl_mtx(infer_mutex_id);
}

/** =================================================================*
 * @brief  音響特徴量完成イベント待ちタスク
 * @param[in] stacd μT-Kernel起動コード（未使用）
 * @param[in] exinf μT-Kernel拡張情報（未使用）
 * ================================================================= */
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
