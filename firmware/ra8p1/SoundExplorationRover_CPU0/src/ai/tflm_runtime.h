/** =================================================================*
 * @file   tflm_runtime.h
 * @brief  CPU0 int8推論ランタイムのC境界
 * @author hino.a
 * @date   2026-09
 * ================================================================= */
#ifndef TFLM_RUNTIME_H
#define TFLM_RUNTIME_H
#include <tk/tkernel.h>                                     /* μT-Kernelの型とリンケージ定義 */

#define TFLM_RUNTIME_ARENA_BYTES            (96U * 1024U)
#define TFLM_RUNTIME_OK                     (0)
#define TFLM_RUNTIME_ARGUMENT_ERROR         (-1)
#define TFLM_RUNTIME_MODEL_ERROR            (-2)
#define TFLM_RUNTIME_ALLOCATION_ERROR       (-3)
#define TFLM_RUNTIME_NOT_READY              (-4)
#define TFLM_RUNTIME_INVOKE_ERROR           (-5)

typedef struct {
    UW input_bytes;
    UW output_bytes;
    UW arena_used_bytes;
    float input_scale;
    float output_scale;
    W input_zero_point;
    W output_zero_point;
} tflm_runtime_info_t;

#ifdef __cplusplus
extern "C" {
#endif

IMPORT INT tflm_runtime_init(UB const * model_data, UW model_bytes); /* 静的モデルから初期化 */
IMPORT INT tflm_runtime_invoke(B const * input, UW input_bytes, B * output, UW output_bytes); /* int8推論 */
IMPORT INT tflm_runtime_get_info(tflm_runtime_info_t * info); /* 入出力量子化とアリーナ実使用量 */
IMPORT void tflm_runtime_reset(void);                        /* 推論状態の破棄 */

#ifdef __cplusplus
}
#endif
#endif /* TFLM_RUNTIME_H */
