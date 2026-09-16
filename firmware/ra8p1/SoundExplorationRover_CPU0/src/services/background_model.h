/** =================================================================*
 * @file   background_model.h
 * @brief  固定乱数エンコーダとRLSデコーダによる音響背景モデル
 * ================================================================= */
#ifndef SEROV_CPU0_BACKGROUND_MODEL_H
#define SEROV_CPU0_BACKGROUND_MODEL_H

#include <tk/tkernel.h>                                     /* μT-Kernel型と公開範囲マクロ */

#define CPU0_BACKGROUND_MODEL_INPUT_DIMENSION       (32U)
#define CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION      (16U)
#define CPU0_BACKGROUND_MODEL_DEFAULT_SEED    (0x53455231U)
#define CPU0_BACKGROUND_MODEL_FORGETTING_FACTOR       (0.95F)
#define CPU0_BACKGROUND_MODEL_INITIAL_DIAGONAL       (100.0F)

typedef struct st_background_model_state {
    float decoder[CPU0_BACKGROUND_MODEL_INPUT_DIMENSION][CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION];
    float inverse_correlation[CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION][CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION];
    UW encoder_seed;
    UW mse_count;
    float mse_mean;
    float mse_m2;
} background_model_state_t;

/* decoder=0、P=I/0.01、固定乱数seedで背景モデルを初期化する。 */
EXPORT void background_model_init(background_model_state_t * p_state, UW encoder_seed);
/* 現在のデコーダによる正規化済み入力の再構成MSEを計算する。 */
EXPORT BOOL background_model_mse(const background_model_state_t * p_state,
                                 const B * p_frame,
                                 float * p_mse);
/* 能動フレームなら採点だけ、非能動ならRLS更新とMSE統計更新を行う。 */
EXPORT BOOL background_model_observe(background_model_state_t * p_state,
                                     const B * p_frame,
                                     BOOL active_frame,
                                     float * p_mse);
/* 学習済み背景MSEのmean + 3 sigmaを返す。統計不足時はFALSE。 */
EXPORT BOOL background_model_mse_threshold(const background_model_state_t * p_state, float * p_threshold);
/* MRAMからdecoderを復元した後にPだけを初期化して継続学習できるようにする。 */
EXPORT void background_model_reset_inverse_correlation(background_model_state_t * p_state);

#endif /* SEROV_CPU0_BACKGROUND_MODEL_H */
