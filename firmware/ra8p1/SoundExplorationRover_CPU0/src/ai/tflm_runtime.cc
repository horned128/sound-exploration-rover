/** =================================================================*
 * @file   tflm_runtime.cc
 * @brief  静的アリーナによるCPU0 TFLM推論
 * @author hino.a
 * @date   2026-09
 * ================================================================= */
#include "ai/tflm_runtime.h"                                /* アプリへ公開するC API */
#include <cstring>                                         /* テンソルのコピー */
#include <new>                                             /* 静的領域へのplacement new */
#include "flatbuffers/verifier.h"                          /* モデルバッファの境界検証 */
#include "tensorflow/lite/micro/micro_interpreter.h"        /* TFLMインタプリタ */
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h" /* 使用演算の明示登録 */
#include "tensorflow/lite/schema/schema_generated.h"       /* TFLiteモデル形式 */

using RuntimeResolver = tflite::MicroMutableOpResolver<1>;
alignas(16) LOCAL UB s_arena[TFLM_RUNTIME_ARENA_BYTES];       /**< CPU0専有アリーナ */
/**< インタプリタの静的構築領域 */
alignas(tflite::MicroInterpreter) LOCAL UB s_interpreter_storage[sizeof(tflite::MicroInterpreter)];
/**< 演算表の静的構築領域 */
alignas(RuntimeResolver) LOCAL UB s_resolver_storage[sizeof(RuntimeResolver)];
LOCAL tflite::MicroInterpreter * s_interpreter;             /**< placement構築した推論器 */
LOCAL RuntimeResolver * s_resolver;                         /**< 明示的に構築する演算表 */
LOCAL BOOL s_ready;                                         /**< 入出力の検証完了 */

/** =================================================================*
 * @brief  推論状態の破棄
 * @details 全APIは単一タスク専用。並行呼出し不可。モデル領域はresetまで保持する。
 * ================================================================= */
EXPORT void tflm_runtime_reset(void) {
    s_ready = FALSE;
    if (s_interpreter != nullptr) {
        s_interpreter->~MicroInterpreter();
        s_interpreter = nullptr;
    }
    if (s_resolver != nullptr) {
        s_resolver->~RuntimeResolver();
        s_resolver = nullptr;
    }
}

/** =================================================================*
 * @brief  静的モデルから初期化
 * @param[in] model_data 16 byte境界に配置した読取専用モデル（resetまで有効）
 * @param[in] model_bytes モデルの実バイト数
 * @return TFLM_RUNTIMEの結果コード
 * @details 失敗時は旧モデルも無効化する。起動タスクや安全制御には依存しない。
 * ================================================================= */
EXPORT INT tflm_runtime_init(UB const * model_data, UW model_bytes) {
    tflm_runtime_reset();
    if ((model_data == nullptr) || (model_bytes < 8U) ||
        ((reinterpret_cast<uintptr_t>(model_data) & 15U) != 0U)) {
        return TFLM_RUNTIME_ARGUMENT_ERROR;
    }
    flatbuffers::Verifier verifier(model_data, model_bytes);
    if (!tflite::VerifyModelBuffer(verifier)) {
        return TFLM_RUNTIME_MODEL_ERROR;
    }
    auto const * model = tflite::GetModel(model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION || model->subgraphs() == nullptr ||
        model->subgraphs()->size() != 1U) {
        return TFLM_RUNTIME_MODEL_ERROR;
    }
    auto const * graph = model->subgraphs()->Get(0);
    if (graph->inputs() == nullptr || graph->outputs() == nullptr || graph->tensors() == nullptr ||
        graph->inputs()->size() != 1U || graph->outputs()->size() != 1U) {
        return TFLM_RUNTIME_MODEL_ERROR;
    }
    W const input_index = graph->inputs()->Get(0);
    W const output_index = graph->outputs()->Get(0);
    if (input_index < 0 || output_index < 0 || static_cast<UW>(input_index) >= graph->tensors()->size() ||
        static_cast<UW>(output_index) >= graph->tensors()->size() ||
        graph->tensors()->Get(input_index)->type() != tflite::TensorType_INT8 ||
        graph->tensors()->Get(output_index)->type() != tflite::TensorType_INT8) {
        return TFLM_RUNTIME_MODEL_ERROR;
    }
    s_resolver = new (s_resolver_storage) RuntimeResolver();
    if (s_resolver->AddFullyConnected() != kTfLiteOk) {
        tflm_runtime_reset();
        return TFLM_RUNTIME_MODEL_ERROR;
    }
    s_interpreter = new (s_interpreter_storage) tflite::MicroInterpreter(
        model, *s_resolver, s_arena, sizeof(s_arena));
    if (s_interpreter->AllocateTensors() != kTfLiteOk) {
        tflm_runtime_reset();
        return TFLM_RUNTIME_ALLOCATION_ERROR;
    }
    if (s_interpreter->inputs_size() != 1U || s_interpreter->outputs_size() != 1U ||
        s_interpreter->input(0)->type != kTfLiteInt8 || s_interpreter->output(0)->type != kTfLiteInt8 ||
        s_interpreter->input(0)->bytes == 0U || s_interpreter->output(0)->bytes == 0U ||
        !(s_interpreter->input(0)->params.scale > 0.0F) || !(s_interpreter->output(0)->params.scale > 0.0F)) {
        tflm_runtime_reset();
        return TFLM_RUNTIME_MODEL_ERROR;
    }
    s_ready = TRUE;
    return TFLM_RUNTIME_OK;
}

/** =================================================================*
 * @brief  int8推論
 * @param[in] input モデルのscaleとzero pointで量子化済みの入力
 * @param[in] input_bytes 入力バイト数（テンソルと完全一致）
 * @param[out] output 成功時のみ書き込む出力
 * @param[in] output_bytes 出力バイト数（テンソルと完全一致）
 * @return TFLM_RUNTIMEの結果コード
 * ================================================================= */
EXPORT INT tflm_runtime_invoke(B const * input, UW input_bytes, B * output, UW output_bytes) {
    if (!s_ready) {
        return TFLM_RUNTIME_NOT_READY;
    }
    if (input == nullptr || output == nullptr || input_bytes != s_interpreter->input(0)->bytes ||
        output_bytes != s_interpreter->output(0)->bytes) {
        return TFLM_RUNTIME_ARGUMENT_ERROR;
    }
    std::memcpy(s_interpreter->input(0)->data.int8, input, input_bytes);
    if (s_interpreter->Invoke() != kTfLiteOk) {
        return TFLM_RUNTIME_INVOKE_ERROR;
    }
    std::memcpy(output, s_interpreter->output(0)->data.int8, output_bytes);
    return TFLM_RUNTIME_OK;
}

/** =================================================================*
 * @brief  入出力量子化とアリーナ実使用量
 * @param[out] info 初期化済みモデルの情報
 * @return TFLM_RUNTIMEの結果コード
 * ================================================================= */
EXPORT INT tflm_runtime_get_info(tflm_runtime_info_t * info) {
    if (info == nullptr) {
        return TFLM_RUNTIME_ARGUMENT_ERROR;
    }
    std::memset(info, 0, sizeof(*info));
    if (!s_ready) {
        return TFLM_RUNTIME_NOT_READY;
    }
    info->input_bytes = static_cast<UW>(s_interpreter->input(0)->bytes);
    info->output_bytes = static_cast<UW>(s_interpreter->output(0)->bytes);
    info->arena_used_bytes = static_cast<UW>(s_interpreter->arena_used_bytes());
    info->input_scale = s_interpreter->input(0)->params.scale;
    info->output_scale = s_interpreter->output(0)->params.scale;
    info->input_zero_point = s_interpreter->input(0)->params.zero_point;
    info->output_zero_point = s_interpreter->output(0)->params.zero_point;
    return TFLM_RUNTIME_OK;
}
