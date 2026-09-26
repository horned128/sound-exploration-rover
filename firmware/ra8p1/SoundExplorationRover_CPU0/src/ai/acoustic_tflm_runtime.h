/** =================================================================*
 * @file   acoustic_tflm_runtime.h
 * @brief  音響埋め込みCNN TFLM推論ランタイムC境界
 * ================================================================= */
#ifndef ACOUSTIC_TFLM_RUNTIME_H
#define ACOUSTIC_TFLM_RUNTIME_H

#include <tk/tkernel.h>                                     /* μT-Kernelの型とリンケージ定義 */

#define ACOUSTIC_TFLM_ARENA_BYTES          (128U * 1024U)   /**< 音響TFLMアリーナ[byte] (実モデル中間テンソルに112KiB以上必要) */
#define ACOUSTIC_TFLM_OK                   (0)              /**< 成功 */
#define ACOUSTIC_TFLM_ARGUMENT_ERROR       (-1)             /**< 引数異常 */
#define ACOUSTIC_TFLM_MODEL_ERROR          (-2)             /**< モデル異常 */
#define ACOUSTIC_TFLM_ALLOCATION_ERROR     (-3)             /**< アリーナ確保異常 */
#define ACOUSTIC_TFLM_NOT_READY            (-4)             /**< 未初期化 */
#define ACOUSTIC_TFLM_INVOKE_ERROR         (-5)             /**< 推論実行異常 */

#ifdef __cplusplus
extern "C" {
#endif

IMPORT INT acoustic_tflm_runtime_init(UB const * model_data, UW model_bytes); /* TFLM初期化 */
/* 80フレーム×32ビンlog-melからの64次元埋め込み推論実行 */
IMPORT INT acoustic_tflm_runtime_invoke(
    B const * input_mel,
    UW input_bytes,
    float * output_embedding,
    UW output_floats);
IMPORT void acoustic_tflm_runtime_reset(void);              /* TFLMリセット */

#ifdef __cplusplus
}
#endif

#endif /* ACOUSTIC_TFLM_RUNTIME_H */
