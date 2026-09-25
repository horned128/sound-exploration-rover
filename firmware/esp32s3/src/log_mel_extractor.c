/** =================================================================*
 * @file   log_mel_extractor.c
 * @brief  ストリーミングlog-mel特徴量抽出
 * ================================================================= */
#include "log_mel_extractor.h"                            /* log-mel抽出状態と固定値 */
#include <limits.h>                                         /* PCM正規化上限 */
#include <math.h>                                           /* 窓、mel、対数演算 */
#include <string.h>                                         /* PCM繰越しコピー */

#define LOG_MEL_LOW_HZ                    (125.0)
#define LOG_MEL_HIGH_HZ                   (8000.0)
#define LOG_MEL_ENERGY_FLOOR              (1.0e-12F)
#define LOG_MEL_QUANTIZATION_SCALE        (0.125F)
#define LOG_MEL_PI                        (3.14159265358979323846)

/**< index 137のインパルスに対する固定int8期待値 */
static int8_t const s_log_mel_self_test_expected[LOG_MEL_BIN_COUNT] = {
    -9, -8, -8, -7, -7, -6, -5, -5, -4, -4, -3, -3, -2, -1, -1, 0,
    0,  1,  1,  2,  3,  3,  4,  4,  5,  5,  6,  7,  7,  8,  8, 9,
};

static double log_mel_hz_to_htk_mel(double frequency_hz);   /* HzからHTK melへ変換 */
static double log_mel_htk_mel_to_hz(double mel);            /* HTK melからHzへ変換 */
static size_t log_mel_reverse_bits(size_t value);            /* 9-bit反転 */
static int8_t log_mel_quantize(float value);                 /* 対称int8量子化 */
static void log_mel_fft(log_mel_extractor_t * extractor);    /* 512点FFT */

/** =================================================================*
 * @brief  HzからHTK melへの変換
 * @param[in] frequency_hz 周波数[Hz]
 * @return HTK mel値
 * ================================================================= */
static double log_mel_hz_to_htk_mel(double frequency_hz) {
    return 2595.0 * log10(1.0 + (frequency_hz / 700.0));
}

/** =================================================================*
 * @brief  HTK melからHzへの変換
 * @param[in] mel HTK mel値
 * @return 周波数[Hz]
 * ================================================================= */
static double log_mel_htk_mel_to_hz(double mel) {
    return 700.0 * (pow(10.0, mel / 2595.0) - 1.0);
}

/** =================================================================*
 * @brief  FFT添字のビット反転
 * @param[in] value 0から511の添字
 * @return 9-bitを反転した添字
 * ================================================================= */
static size_t log_mel_reverse_bits(size_t value) {
    size_t reversed = 0U;
    for (size_t bit = 0U; bit < 9U; bit++) {
        reversed = (reversed << 1U) | (value & 1U);
        value >>= 1U;
    }
    return reversed;
}

/** =================================================================*
 * @brief  対称int8量子化
 * @param[in] value フレーム平均減算後のlog-mel値
 * @return scale 0.125、zero point 0で量子化した値
 * ================================================================= */
static int8_t log_mel_quantize(float value) {
    float const scaled = value / LOG_MEL_QUANTIZATION_SCALE;
    long quantized = lroundf(scaled);
    if (quantized > INT8_MAX) {
        quantized = INT8_MAX;
    } else if (quantized < INT8_MIN) {
        quantized = INT8_MIN;
    }
    return (int8_t) quantized;
}

/** =================================================================*
 * @brief  512点非正規化FFT
 * @param[in,out] extractor FFT作業領域と回転因子
 * ================================================================= */
static void log_mel_fft(log_mel_extractor_t * extractor) {
    for (size_t index = 0U; index < LOG_MEL_FFT_SIZE; index++) {
        size_t const reversed = log_mel_reverse_bits(index);
        if (reversed > index) {
            float temporary = extractor->fft_real[index];
            extractor->fft_real[index] = extractor->fft_real[reversed];
            extractor->fft_real[reversed] = temporary;
            temporary = extractor->fft_imag[index];
            extractor->fft_imag[index] = extractor->fft_imag[reversed];
            extractor->fft_imag[reversed] = temporary;
        }
    }

    for (size_t length = 2U; length <= LOG_MEL_FFT_SIZE; length <<= 1U) {
        size_t const half_length = length / 2U;
        size_t const twiddle_step = LOG_MEL_FFT_SIZE / length;
        for (size_t base = 0U; base < LOG_MEL_FFT_SIZE; base += length) {
            for (size_t offset = 0U; offset < half_length; offset++) {
                size_t const twiddle_index = offset * twiddle_step;
                float const odd_real = extractor->fft_real[base + offset + half_length];
                float const odd_imag = extractor->fft_imag[base + offset + half_length];
                float const rotated_real = (extractor->twiddle_real[twiddle_index] * odd_real) -
                                           (extractor->twiddle_imag[twiddle_index] * odd_imag);
                float const rotated_imag = (extractor->twiddle_real[twiddle_index] * odd_imag) +
                                           (extractor->twiddle_imag[twiddle_index] * odd_real);
                float const even_real = extractor->fft_real[base + offset];
                float const even_imag = extractor->fft_imag[base + offset];
                extractor->fft_real[base + offset] = even_real + rotated_real;
                extractor->fft_imag[base + offset] = even_imag + rotated_imag;
                extractor->fft_real[base + offset + half_length] = even_real - rotated_real;
                extractor->fft_imag[base + offset + half_length] = even_imag - rotated_imag;
            }
        }
    }
}

/** =================================================================*
 * @brief  特徴量抽出状態サイズ取得
 * @return log_mel_extractor_tのバイト数
 * ================================================================= */
size_t log_mel_extractor_context_size(void) {
    return sizeof(log_mel_extractor_t);
}

/** =================================================================*
 * @brief  特徴量抽出器初期化
 * @param[out] extractor 初期化する抽出状態
 * ================================================================= */
void log_mel_extractor_init(log_mel_extractor_t * extractor) {
    if (extractor == NULL) {
        return;
    }

    memset(extractor, 0, sizeof(*extractor));
    for (size_t index = 0U; index < LOG_MEL_WINDOW_SAMPLES; index++) {
        double const phase = (2.0 * LOG_MEL_PI * (double) index) / (double) LOG_MEL_WINDOW_SAMPLES;
        extractor->hann_window[index] = (float) (0.5 - (0.5 * cos(phase)));
    }
    for (size_t index = 0U; index < (LOG_MEL_FFT_SIZE / 2U); index++) {
        double const phase = (-2.0 * LOG_MEL_PI * (double) index) / (double) LOG_MEL_FFT_SIZE;
        extractor->twiddle_real[index] = (float) cos(phase);
        extractor->twiddle_imag[index] = (float) sin(phase);
    }

    double const mel_low = log_mel_hz_to_htk_mel(LOG_MEL_LOW_HZ);
    double const mel_high = log_mel_hz_to_htk_mel(LOG_MEL_HIGH_HZ);
    for (size_t index = 0U; index < LOG_MEL_EDGE_COUNT; index++) {
        double const ratio = (double) index / (double) (LOG_MEL_EDGE_COUNT - 1U);
        extractor->mel_edges_hz[index] = (float) log_mel_htk_mel_to_hz(mel_low + (ratio * (mel_high - mel_low)));
    }
    extractor->initialized = true;
}

/** =================================================================*
 * @brief  PCM繰越し状態初期化
 * @param[in,out] extractor 抽出状態
 * ================================================================= */
void log_mel_extractor_reset(log_mel_extractor_t * extractor) {
    if (extractor == NULL) {
        return;
    }
    extractor->buffered_samples = 0U;
}

/** =================================================================*
 * @brief  固定ベクトル自己診断
 * @param[in,out] extractor 初期化済み抽出状態
 * @return 実行環境上のint8出力が固定期待値と一致すればtrue
 * @details FFT、mel、対数、量子化を通し、演算系または係数生成の差異を起動時に検出する。
 * ================================================================= */
bool log_mel_extractor_self_test(log_mel_extractor_t * extractor) {
    if ((extractor == NULL) || !extractor->initialized) {
        return false;
    }

    int32_t samples[LOG_MEL_WINDOW_SAMPLES] = {0};
    int8_t output[LOG_MEL_BIN_COUNT];
    samples[137] = (int32_t) (1UL << 30U);
    log_mel_extractor_process_window(extractor, samples, output);
    log_mel_extractor_reset(extractor);
    return memcmp(output, s_log_mel_self_test_expected, sizeof(output)) == 0;
}

/** =================================================================*
 * @brief  1窓のlog-mel特徴量抽出
 * @param[in,out] extractor 係数と作業領域
 * @param[in] samples 16 kHz mono PCM 400サンプル
 * @param[out] output 32 bin int8特徴量
 * ================================================================= */
void log_mel_extractor_process_window(log_mel_extractor_t * extractor,
                                       int32_t const samples[LOG_MEL_WINDOW_SAMPLES],
                                       int8_t output[LOG_MEL_BIN_COUNT]) {
    log_mel_extractor_process_window_pair(extractor, samples, output, NULL);
}

/** =================================================================*
 * @brief centeredログメルと音量保持ログメルを1回のFFTから計算
 * @details absoluteのみ必要な場合はcenteredをNULLで呼べる。旧通信契約は不変。
 * ================================================================= */
void log_mel_extractor_process_window_pair(log_mel_extractor_t * extractor,
                                           int32_t const samples[LOG_MEL_WINDOW_SAMPLES],
                                           int8_t centered[LOG_MEL_BIN_COUNT],
                                           int8_t absolute[LOG_MEL_BIN_COUNT]) {
    if ((extractor == NULL) || (samples == NULL) ||
        ((centered == NULL) && (absolute == NULL)) || !extractor->initialized) {
        return;
    }

    for (size_t index = 0U; index < LOG_MEL_WINDOW_SAMPLES; index++) {
        float const normalized = (float) samples[index] / (float) INT32_MAX;
        extractor->fft_real[index] = normalized * extractor->hann_window[index];
        extractor->fft_imag[index] = 0.0F;
    }
    for (size_t index = LOG_MEL_WINDOW_SAMPLES; index < LOG_MEL_FFT_SIZE; index++) {
        extractor->fft_real[index] = 0.0F;
        extractor->fft_imag[index] = 0.0F;
    }

    log_mel_fft(extractor);
    for (size_t index = 0U; index < LOG_MEL_SPECTRUM_BINS; index++) {
        float const real = extractor->fft_real[index];
        float const imag = extractor->fft_imag[index];
        extractor->power[index] = (real * real) + (imag * imag);
    }

    float log_energy[LOG_MEL_BIN_COUNT];
    float mean = 0.0F;
    for (size_t mel_bin = 0U; mel_bin < LOG_MEL_BIN_COUNT; mel_bin++) {
        float const left_hz = extractor->mel_edges_hz[mel_bin];
        float const center_hz = extractor->mel_edges_hz[mel_bin + 1U];
        float const right_hz = extractor->mel_edges_hz[mel_bin + 2U];
        float energy = 0.0F;

        for (size_t spectrum_bin = 0U; spectrum_bin < LOG_MEL_SPECTRUM_BINS; spectrum_bin++) {
            float const frequency_hz = ((float) spectrum_bin * (float) LOG_MEL_SAMPLE_RATE_HZ) /
                                       (float) LOG_MEL_FFT_SIZE;
            float weight = 0.0F;
            if ((frequency_hz >= left_hz) && (frequency_hz < center_hz)) {
                weight = (frequency_hz - left_hz) / (center_hz - left_hz);
            } else if ((frequency_hz >= center_hz) && (frequency_hz <= right_hz)) {
                weight = (right_hz - frequency_hz) / (right_hz - center_hz);
            }
            energy += weight * extractor->power[spectrum_bin];
        }

        if (energy < LOG_MEL_ENERGY_FLOOR) {
            energy = LOG_MEL_ENERGY_FLOOR;
        }
        log_energy[mel_bin] = logf(energy);
        mean += log_energy[mel_bin];
    }
    mean /= (float) LOG_MEL_BIN_COUNT;

    for (size_t mel_bin = 0U; mel_bin < LOG_MEL_BIN_COUNT; mel_bin++) {
        if (centered != NULL) {
            centered[mel_bin] = log_mel_quantize(log_energy[mel_bin] - mean);
        }
        if (absolute != NULL) {
            absolute[mel_bin] = log_mel_quantize(log_energy[mel_bin] + 8.0F);
        }
    }
}

/** =================================================================*
 * @brief  PCMストリームから特徴量フレーム生成
 * @param[in,out] extractor 抽出状態
 * @param[in] samples 16 kHz mono PCM
 * @param[in] sample_count PCMサンプル数
 * @param[in] callback 生成フレーム通知先。NULLなら通知しない
 * @param[in] context 通知先コンテキスト
 * @return 生成した特徴量フレーム数
 * ================================================================= */
size_t log_mel_extractor_feed(log_mel_extractor_t * extractor, int32_t const * samples, size_t sample_count,
                              log_mel_frame_callback_t callback, void * context) {
    if ((extractor == NULL) || (samples == NULL) || !extractor->initialized) {
        return 0U;
    }

    size_t input_index = 0U;
    size_t generated_frames = 0U;
    while (input_index < sample_count) {
        size_t const needed = LOG_MEL_WINDOW_SAMPLES - extractor->buffered_samples;
        size_t const remaining = sample_count - input_index;
        size_t const copy_count = (remaining < needed) ? remaining : needed;
        memcpy(&extractor->sample_buffer[extractor->buffered_samples], &samples[input_index],
               copy_count * sizeof(samples[0]));
        extractor->buffered_samples += copy_count;
        input_index += copy_count;

        if (extractor->buffered_samples == LOG_MEL_WINDOW_SAMPLES) {
            int8_t frame[LOG_MEL_BIN_COUNT];
            log_mel_extractor_process_window(extractor, extractor->sample_buffer, frame);
            if (callback != NULL) {
                callback(frame, context);
            }
            generated_frames++;
            memmove(extractor->sample_buffer, &extractor->sample_buffer[LOG_MEL_HOP_SAMPLES],
                    (LOG_MEL_WINDOW_SAMPLES - LOG_MEL_HOP_SAMPLES) * sizeof(extractor->sample_buffer[0]));
            extractor->buffered_samples = LOG_MEL_WINDOW_SAMPLES - LOG_MEL_HOP_SAMPLES;
        }
    }
    return generated_frames;
}
