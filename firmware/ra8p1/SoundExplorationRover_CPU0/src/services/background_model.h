/** =================================================================*
 * @file   background_model.h
 * @brief  固定乱数エンコーダとRLSデコーダによる音響背景モデル
 * ================================================================= */
#ifndef SEROV_CPU0_BACKGROUND_MODEL_H
#define SEROV_CPU0_BACKGROUND_MODEL_H

#include <tk/tkernel.h>                                     /* μT-Kernel型と公開範囲マクロ */

#define CPU0_BACKGROUND_MODEL_INPUT_DIMENSION (32U)         /**< 背景モデル入力ベクトルの次元数 */
#define CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION (16U)        /**< 背景モデル隠れ層の次元数 */
#define CPU0_BACKGROUND_MODEL_DEFAULT_SEED (0x53455231U)    /**< 背景モデル乱数生成の既定シード */
#define CPU0_BACKGROUND_MODEL_FORGETTING_FACTOR (0.95F)     /**< 背景モデルRLSの忘却係数 */
#define CPU0_BACKGROUND_MODEL_INITIAL_DIAGONAL (100.0F)     /**< 背景モデル逆相関行列の初期対角値 */

/**< 固定乱数エンコーダとRLS背景モデルの学習状態 */
typedef struct st_background_model_state {
    /**< 入力を隠れ層へ写像するデコーダ行列 */
    float decoder[CPU0_BACKGROUND_MODEL_INPUT_DIMENSION][CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION];
    /**< RLS更新で保持する隠れ層逆相関行列 */
    float inverse_correlation[CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION][CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION];
    UW encoder_seed;                                        /**< 固定乱数エンコーダのseed */
    UW mse_count;                                           /**< 背景MSEの有効サンプル数 */
    float mse_mean;                                         /**< 背景MSEの平均値 */
    float mse_m2;                                           /**< 背景MSEの分散計算用二次モーメント */
} background_model_state_t;

/* decoder=0、P=I/0.01、固定乱数seedで背景モデルを初期化する。 */
EXPORT void background_model_init(background_model_state_t * p_state, UW encoder_seed); /* 背景モデル初期化 */
EXPORT BOOL background_model_mse(const background_model_state_t * p_state,
                                 const B * p_frame,
                                 float * p_mse); /* 正規化フレームの再構成MSE算出 */
EXPORT BOOL background_model_observe(background_model_state_t * p_state,
                                     const B * p_frame,
                                     BOOL active_frame,
                                     float * p_mse); /* 背景モデル観測・RLS更新 */
/* 学習済み背景MSEのmean + 3 sigmaを返す。統計不足時はFALSE。 */
EXPORT BOOL background_model_mse_threshold(const background_model_state_t * p_state,
                                          float * p_threshold); /* MSEしきい値 */
/* MRAMからdecoderを復元した後にPだけを初期化して継続学習できるようにする。 */
EXPORT void background_model_reset_inverse_correlation(background_model_state_t * p_state); /* 逆相関初期化 */

#endif /* SEROV_CPU0_BACKGROUND_MODEL_H */
