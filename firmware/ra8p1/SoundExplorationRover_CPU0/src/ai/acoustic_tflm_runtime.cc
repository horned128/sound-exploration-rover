/** =================================================================*
 * @file   acoustic_tflm_runtime.cc
 * @brief  静的アリーナによるCPU0 音響埋め込みCNN TFLM推論
 * ================================================================= */
#include "ai/acoustic_tflm_runtime.h"                       /* ランタイムC境界API */
#include <cstring>                                          /* std::memcpy */
#include <new>                                              /* 配置new */
#include "flatbuffers/verifier.h"                           /* FlatBuffers検証 */
#include "tensorflow/lite/micro/micro_interpreter.h"        /* TFLMインタプリタ */
/* 演算子リゾルバ */
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"        /* TFLiteスキーマ定義 */

using AcousticResolver = tflite::MicroMutableOpResolver<20>;

/**< 音響TFLMテンソルアリーナ (静的確保) */
alignas(16) LOCAL UB s_acoustic_arena[ACOUSTIC_TFLM_ARENA_BYTES];
/**< インタプリタ配置用静的バッファ */
alignas(tflite::MicroInterpreter) LOCAL UB s_acoustic_interpreter_storage[sizeof(tflite::MicroInterpreter)];
/**< 演算子リゾルバ配置用静的バッファ */
alignas(AcousticResolver) LOCAL UB s_acoustic_resolver_storage[sizeof(AcousticResolver)];

LOCAL tflite::MicroInterpreter * s_acoustic_interpreter = nullptr; /**< インタプリタポインタ */
LOCAL AcousticResolver * s_acoustic_resolver = nullptr;     /**< 演算子リゾルバポインタ */
LOCAL BOOL s_acoustic_ready = FALSE;                        /**< 推論実行可能状態 */

/** =================================================================*
 * @brief  音響TFLM推論ランタイムリセット
 * ================================================================= */
EXPORT void acoustic_tflm_runtime_reset(void) {
    s_acoustic_ready = FALSE;
    if (s_acoustic_interpreter != nullptr) {
        s_acoustic_interpreter->~MicroInterpreter();
        s_acoustic_interpreter = nullptr;
    }
    if (s_acoustic_resolver != nullptr) {
        s_acoustic_resolver->~AcousticResolver();
        s_acoustic_resolver = nullptr;
    }
}

/** =================================================================*
 * @brief  音響TFLM推論ランタイム初期化
 * @param[in] model_data TFLiteモデルバイナリ先頭アドレス
 * @param[in] model_bytes モデルバイナリ長[byte]
 * @return 処理結果コード (ACOUSTIC_TFLM_OK等)
 * ================================================================= */
EXPORT INT acoustic_tflm_runtime_init(UB const * model_data, UW model_bytes) {
    acoustic_tflm_runtime_reset();
    if ((model_data == nullptr) || (model_bytes < 8U) ||
        ((reinterpret_cast<uintptr_t>(model_data) & 15U) != 0U)) {
        return ACOUSTIC_TFLM_ARGUMENT_ERROR;
    }

    flatbuffers::Verifier verifier(model_data, model_bytes);
    if (!tflite::VerifyModelBuffer(verifier)) {
        return ACOUSTIC_TFLM_MODEL_ERROR;
    }

    auto const * model = tflite::GetModel(model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION || model->subgraphs() == nullptr ||
        model->subgraphs()->size() != 1U) {
        return ACOUSTIC_TFLM_MODEL_ERROR;
    }

    s_acoustic_resolver = new (s_acoustic_resolver_storage) AcousticResolver();
    (void) s_acoustic_resolver->AddConv2D();
    (void) s_acoustic_resolver->AddDepthwiseConv2D();
    (void) s_acoustic_resolver->AddMaxPool2D();
    (void) s_acoustic_resolver->AddMean();
    (void) s_acoustic_resolver->AddFullyConnected();
    (void) s_acoustic_resolver->AddRelu();
    (void) s_acoustic_resolver->AddRelu6();
    (void) s_acoustic_resolver->AddL2Normalization();
    (void) s_acoustic_resolver->AddReshape();
    (void) s_acoustic_resolver->AddDequantize();
    (void) s_acoustic_resolver->AddSquare();
    (void) s_acoustic_resolver->AddQuantize();
    (void) s_acoustic_resolver->AddSum();
    (void) s_acoustic_resolver->AddRsqrt();
    (void) s_acoustic_resolver->AddMinimum();
    (void) s_acoustic_resolver->AddMul();

    s_acoustic_interpreter = new (s_acoustic_interpreter_storage) tflite::MicroInterpreter(
        model, *s_acoustic_resolver, s_acoustic_arena, sizeof(s_acoustic_arena));

    if (s_acoustic_interpreter->AllocateTensors() != kTfLiteOk) {
        acoustic_tflm_runtime_reset();
        return ACOUSTIC_TFLM_ALLOCATION_ERROR;
    }

    if (s_acoustic_interpreter->inputs_size() != 1U || s_acoustic_interpreter->outputs_size() != 1U) {
        acoustic_tflm_runtime_reset();
        return ACOUSTIC_TFLM_MODEL_ERROR;
    }

    s_acoustic_ready = TRUE;
    return ACOUSTIC_TFLM_OK;
}

/** =================================================================*
 * @brief  80フレーム×32ビンlog-melからの64次元埋め込み推論実行
 * @param[in]  input_mel 80フレーム×32ビンlog-mel入力配列
 * @param[in]  input_bytes 入力バイト数
 * @param[out] output_embedding 出力64次元float埋め込みバッファ
 * @param[in]  output_floats 出力配列要素数
 * @return 処理結果コード (ACOUSTIC_TFLM_OK等)
 * ================================================================= */
EXPORT INT acoustic_tflm_runtime_invoke(
    B const * input_mel,
    UW input_bytes,
    float * output_embedding,
    UW output_floats)
{
    if (!s_acoustic_ready || (s_acoustic_interpreter == nullptr)) {
        return ACOUSTIC_TFLM_NOT_READY;
    }
    if ((input_mel == nullptr) || (output_embedding == nullptr)) {
        return ACOUSTIC_TFLM_ARGUMENT_ERROR;
    }

    TfLiteTensor * input_tensor = s_acoustic_interpreter->input(0);
    TfLiteTensor * output_tensor = s_acoustic_interpreter->output(0);

    if (input_bytes != input_tensor->bytes ||
        (output_floats * sizeof(float)) != output_tensor->bytes) {
        return ACOUSTIC_TFLM_ARGUMENT_ERROR;
    }

    std::memcpy(input_tensor->data.int8, input_mel, input_bytes);

    if (s_acoustic_interpreter->Invoke() != kTfLiteOk) {
        return ACOUSTIC_TFLM_INVOKE_ERROR;
    }

    std::memcpy(output_embedding, output_tensor->data.f, output_tensor->bytes);
    return ACOUSTIC_TFLM_OK;
}
