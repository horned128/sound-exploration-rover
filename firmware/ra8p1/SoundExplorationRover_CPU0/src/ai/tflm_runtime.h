/** =================================================================*
 * @file   tflm_runtime.h
 * @brief  CPU0 int8推論ランタイムのC境界
 * @author hino.a
 * @date   2026-09
 * ================================================================= */
#ifndef TFLM_RUNTIME_H
#define TFLM_RUNTIME_H
#include <tk/tkernel.h>                                     /* μT-Kernelの型とリンケージ定義 */

#define TFLM_RUNTIME_ARENA_BYTES           (96U * 1024U)    /**< TFLM実行環境のアリーナ[byte] */
#define TFLM_RUNTIME_OK                    (0)              /**< TFLM推論実行の成功コード */
#define TFLM_RUNTIME_ARGUMENT_ERROR        (-1)             /**< TFLM推論実行の引数異常コード */
#define TFLM_RUNTIME_MODEL_ERROR           (-2)             /**< TFLMモデル読込み異常コード */
#define TFLM_RUNTIME_ALLOCATION_ERROR      (-3)             /**< TFLM推論領域確保異常コード */
#define TFLM_RUNTIME_NOT_READY             (-4)             /**< TFLM推論準備未完了コード */
#define TFLM_RUNTIME_INVOKE_ERROR          (-5)             /**< TFLM推論実行異常コード */

/**< TFLM入出力テンソルと推論領域の実行情報 */
typedef struct {
    UW input_bytes;                                         /**< 入力テンソルのバイト数 */
    UW output_bytes;                                        /**< 出力テンソルのバイト数 */
    UW arena_used_bytes;                                    /**< 使用中のテンソルアリーナ容量[byte] */
    float input_scale;                                      /**< 入力量子化スケール */
    float output_scale;                                     /**< 出力量子化スケール */
    W input_zero_point;                                     /**< 入力量子化ゼロ点 */
    W output_zero_point;                                    /**< 出力量子化ゼロ点 */
} tflm_runtime_info_t;

#ifdef __cplusplus
extern "C" {
#endif

IMPORT INT tflm_runtime_init(UB const * model_data, UW model_bytes); /* 静的モデルから初期化 */
IMPORT INT tflm_runtime_invoke(B const * input, UW input_bytes, B * output, UW output_bytes); /* int8推論 */
IMPORT INT tflm_runtime_get_info(tflm_runtime_info_t * info); /* 入出力量子化とアリーナ実使用量 */
IMPORT void tflm_runtime_reset(void);                       /* 推論状態の破棄 */

#ifdef __cplusplus
}
#endif
#endif /* TFLM_RUNTIME_H */
