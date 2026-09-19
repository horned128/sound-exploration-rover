/** =================================================================*
 * @file   task_infer.h
 * @brief  CPU0音響背景学習・現場見本照合タスクAPI
 * ================================================================= */
#ifndef SEROV_CPU0_TASK_INFER_H
#define SEROV_CPU0_TASK_INFER_H

#include "services/acoustic_identifier.h"                  /* 要約識別の結果型 */
#include "services/prototype_storage.h"                    /* 背景モデル・見本の保存型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

typedef struct st_task_infer_result {
    UW feature_generation;                                  /**< 処理した完成特徴量世代 */
    UB active_frame_count;                                  /**< 80frame中の能動フレーム数 */
    BOOL summary_valid;                                     /**< N_min以上で96次元要約が有効 */
    BOOL background_threshold_valid;                        /**< 背景MSEしきい値が学習済み */
    BOOL background_anomaly;                                /**< 背景候補として異常ならTRUE */
    float event_mse;                                        /**< 能動フレームの最大再構成MSE */
    float background_threshold;                             /**< 背景MSE mean+3sigma */
    B summary[CPU0_ACOUSTIC_SUMMARY_DIMENSION];             /**< 2スロット分割mean/std/maxの192次元int8 */
    acoustic_identifier_summary_output_t identifier;        /**< 個別見本へのcosine照合結果 */
} task_infer_result_t;

typedef struct st_task_infer_prototype_telemetry {
    BOOL storage_valid;
    UB sample_count;
    UB target_peak_bin;
    float identifier_threshold;
} task_infer_prototype_telemetry_t;

/* 推論タスクは走行タスク群と独立に、失敗しても非致命で起動する。 */
EXPORT void task_infer_start_optional(void);
EXPORT void task_infer_stop(void);
/* 音響リンクタスクから完成特徴量を通知する。 */
EXPORT ER task_infer_notify_feature_ready(void);
/* 思考タスクがMRAMから読んだ、または保存した背景モデル・見本を更新する。 */
EXPORT ER task_infer_prototype_set(const prototype_storage_data_t * p_data, BOOL storage_valid);
/* 現在の背景デコーダと背景MSEしきい値を保存データへコピーする。 */
EXPORT ER task_infer_background_export(prototype_storage_data_t * p_data);
/* 最新推論結果をコピーする。まだ特徴量を処理していなければE_NOEXS。 */
EXPORT ER task_infer_result_get(task_infer_result_t * p_result);
/* 保存済み見本データを安全に取得する。 */
EXPORT ER task_infer_prototype_get(prototype_storage_data_t * p_data, BOOL * p_storage_valid);
/* テレメトリ用の軽量見本情報取得（スタック消費を抑える） */
EXPORT ER task_infer_prototype_telemetry_get(task_infer_prototype_telemetry_t * p_telemetry);

IMPORT volatile BOOL g_task_infer_available;
IMPORT volatile UW g_task_infer_feature_generation;
IMPORT volatile UW g_task_infer_inference_count;
IMPORT volatile UW g_task_infer_failure_count;
IMPORT volatile UW g_task_infer_match_count;
IMPORT volatile ER g_task_infer_last_kernel_error;

#endif /* SEROV_CPU0_TASK_INFER_H */
