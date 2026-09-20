/** =================================================================*
 * @file   background_model.c
 * @brief  固定乱数32→16エンコーダとRLS 16→32デコーダ
 * ================================================================= */
#include "services/background_model.h"                      /* 背景モデル公開API */
#include <math.h>                                           /* MSE標準偏差 */
#include <string.h>                                         /* 状態初期化 */

LOCAL UW background_model_lcg_next(UW state);               /* 固定乱数系列の1ステップ */
LOCAL float background_model_lcg_uniform(UW * p_state);     /* [-1,1]の固定乱数 */
LOCAL float background_model_hard_sigmoid(float value);     /* clip(0.2x+0.5,0,1) */
LOCAL void background_model_hidden(const background_model_state_t * p_state,
                                   const B * p_frame,
                                   float * p_hidden);
LOCAL void background_model_input_normalize(const B * p_frame, float * p_input); /* 背景モデル入力正規化 */

/** =================================================================*
 * @brief  32-bit LCGを1ステップ進める
 * ================================================================= */
LOCAL UW background_model_lcg_next(UW state) {
    return (UW) ((1664525U * state) + 1013904223U);
}

/** =================================================================*
 * @brief  LCG上位24bitを[-1,1]の固定乱数へ写す
 * ================================================================= */
LOCAL float background_model_lcg_uniform(UW * p_state) {
    *p_state = background_model_lcg_next(*p_state);
    return (((float) (*p_state >> 8U) / 16777215.0F) * 2.0F) - 1.0F;
}

/** =================================================================*
 * @brief  Hard Sigmoidを計算する
 * ================================================================= */
LOCAL float background_model_hard_sigmoid(float value) {
    value = (0.2F * value) + 0.5F;
    if (value < 0.0F) {
        return 0.0F;
    }
    if (value > 1.0F) {
        return 1.0F;
    }
    return value;
}

/** =================================================================*
 * @brief  int8 log-melを±64でclipして[0,1]へ正規化
 * ================================================================= */
LOCAL void background_model_input_normalize(const B * p_frame, float * p_input) {
    for (UW index = 0U; index < CPU0_BACKGROUND_MODEL_INPUT_DIMENSION; index++) {
        W value = (W) p_frame[index];
        if (value < -64) {
            value = -64;
        }
        if (value > 64) {
            value = 64;
        }
        p_input[index] = ((float) value + 64.0F) / 128.0F;
    }
}

/** =================================================================*
 * @brief  seedからエンコーダを都度再生成して隠れ16次元を計算
 * @details エンコーダは状態・MRAMへ保存しない。重み512個の後にbias16個を生成する。
 * ================================================================= */
LOCAL void background_model_hidden(const background_model_state_t * p_state,
                                   const B * p_frame,
                                   float * p_hidden) {
    float input[CPU0_BACKGROUND_MODEL_INPUT_DIMENSION];
    background_model_input_normalize(p_frame, input);
    UW random_state = p_state->encoder_seed;
    for (UW hidden = 0U; hidden < CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION; hidden++) {
        float sum = 0.0F;
        for (UW index = 0U; index < CPU0_BACKGROUND_MODEL_INPUT_DIMENSION; index++) {
            sum += background_model_lcg_uniform(&random_state) * input[index];
        }
        p_hidden[hidden] = sum;
    }
    for (UW hidden = 0U; hidden < CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION; hidden++) {
        float const bias = background_model_lcg_uniform(&random_state) * 0.5F;
        p_hidden[hidden] = background_model_hard_sigmoid(p_hidden[hidden] + bias);
    }
}

/** =================================================================*
 * @brief  decoder=0、P=I/0.01で初期化する
 * ================================================================= */
EXPORT void background_model_init(background_model_state_t * p_state, UW encoder_seed) {
    if (NULL == p_state) {
        return;
    }
    memset(p_state, 0, sizeof(*p_state));
    p_state->encoder_seed = (0U == encoder_seed) ? CPU0_BACKGROUND_MODEL_DEFAULT_SEED : encoder_seed;
    background_model_reset_inverse_correlation(p_state);
}

/** =================================================================*
 * @brief  Pだけを初期化する（decoder復元後の継続学習用）
 * ================================================================= */
EXPORT void background_model_reset_inverse_correlation(background_model_state_t * p_state) {
    if (NULL == p_state) {
        return;
    }
    memset(p_state->inverse_correlation, 0, sizeof(p_state->inverse_correlation));
    for (UW index = 0U; index < CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION; index++) {
        p_state->inverse_correlation[index][index] = CPU0_BACKGROUND_MODEL_INITIAL_DIAGONAL;
    }
}

/** =================================================================*
 * @brief  現在のモデルによる再構成MSEを計算する
 * ================================================================= */
EXPORT BOOL background_model_mse(const background_model_state_t * p_state,
                                 const B * p_frame,
                                 float * p_mse) {
    if ((NULL == p_state) || (NULL == p_frame) || (NULL == p_mse)) {
        return FALSE;
    }
    float input[CPU0_BACKGROUND_MODEL_INPUT_DIMENSION];
    float hidden[CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION];
    background_model_input_normalize(p_frame, input);
    background_model_hidden(p_state, p_frame, hidden);
    float squared_sum = 0.0F;
    for (UW output = 0U; output < CPU0_BACKGROUND_MODEL_INPUT_DIMENSION; output++) {
        float reconstruction = 0.0F;
        for (UW hidden_index = 0U; hidden_index < CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION; hidden_index++) {
            reconstruction += p_state->decoder[output][hidden_index] * hidden[hidden_index];
        }
        float const residual = input[output] - reconstruction;
        squared_sum += residual * residual;
    }
    *p_mse = squared_sum / (float) CPU0_BACKGROUND_MODEL_INPUT_DIMENSION;
    return TRUE;
}

/** =================================================================*
 * @brief  非能動フレームだけをRLSで背景モデルへ取り込む
 * ================================================================= */
EXPORT BOOL background_model_observe(background_model_state_t * p_state,
                                     const B * p_frame,
                                     BOOL active_frame,
                                     float * p_mse) {
    if (!background_model_mse(p_state, p_frame, p_mse)) {
        return FALSE;
    }
    if (active_frame) {
        return TRUE;
    }

    float input[CPU0_BACKGROUND_MODEL_INPUT_DIMENSION];
    float hidden[CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION];
    float p_hidden[CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION];
    float gain[CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION];
    float hidden_p[CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION];
    background_model_input_normalize(p_frame, input);
    background_model_hidden(p_state, p_frame, hidden);
    for (UW row = 0U; row < CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION; row++) {
        p_hidden[row] = 0.0F;
        hidden_p[row] = 0.0F;
        for (UW column = 0U; column < CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION; column++) {
            p_hidden[row] += p_state->inverse_correlation[row][column] * hidden[column];
            hidden_p[row] += hidden[column] * p_state->inverse_correlation[column][row];
        }
    }
    float denominator = CPU0_BACKGROUND_MODEL_FORGETTING_FACTOR;
    for (UW hidden_index = 0U; hidden_index < CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION; hidden_index++) {
        denominator += hidden[hidden_index] * p_hidden[hidden_index];
    }
    for (UW hidden_index = 0U; hidden_index < CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION; hidden_index++) {
        gain[hidden_index] = p_hidden[hidden_index] / denominator;
    }
    for (UW output = 0U; output < CPU0_BACKGROUND_MODEL_INPUT_DIMENSION; output++) {
        float reconstruction = 0.0F;
        for (UW hidden_index = 0U; hidden_index < CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION; hidden_index++) {
            reconstruction += p_state->decoder[output][hidden_index] * hidden[hidden_index];
        }
        float const residual = input[output] - reconstruction;
        for (UW hidden_index = 0U; hidden_index < CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION; hidden_index++) {
            p_state->decoder[output][hidden_index] += residual * gain[hidden_index];
        }
    }
    for (UW row = 0U; row < CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION; row++) {
        for (UW column = 0U; column < CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION; column++) {
            p_state->inverse_correlation[row][column] =
                (p_state->inverse_correlation[row][column] - (gain[row] * hidden_p[column])) /
                CPU0_BACKGROUND_MODEL_FORGETTING_FACTOR;
        }
    }

    p_state->mse_count++;
    float const delta = *p_mse - p_state->mse_mean;
    p_state->mse_mean += delta / (float) p_state->mse_count;
    p_state->mse_m2 += delta * (*p_mse - p_state->mse_mean);
    return TRUE;
}

/** =================================================================*
 * @brief  学習済み背景MSEのmean + 3 sigmaを返す
 * ================================================================= */
EXPORT BOOL background_model_mse_threshold(const background_model_state_t * p_state, float * p_threshold) {
    if ((NULL == p_state) || (NULL == p_threshold) || (p_state->mse_count < 2U)) {
        return FALSE;
    }
    float const variance = p_state->mse_m2 / (float) p_state->mse_count;
    *p_threshold = p_state->mse_mean + (3.0F * sqrtf((variance > 0.0F) ? variance : 0.0F));
    return TRUE;
}
