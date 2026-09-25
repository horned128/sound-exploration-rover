/** =================================================================*
 * @file   log_mel_extractor.h
 * @brief  ストリーミングlog-mel特徴量抽出
 * ================================================================= */
#ifndef RESPEAKER_LOG_MEL_EXTRACTOR_H
#define RESPEAKER_LOG_MEL_EXTRACTOR_H

#include <stdbool.h>                                        /* 真偽値 */
#include <stddef.h>                                         /* size_t */
#include <stdint.h>                                         /* 固定幅整数型 */

#define LOG_MEL_SAMPLE_RATE_HZ             (16000U)
#define LOG_MEL_WINDOW_SAMPLES             (400U)
#define LOG_MEL_HOP_SAMPLES                (160U)
#define LOG_MEL_FFT_SIZE                   (512U)
#define LOG_MEL_SPECTRUM_BINS              (257U)
#define LOG_MEL_BIN_COUNT                  (32U)
#define LOG_MEL_EDGE_COUNT                 (34U)

typedef void (*log_mel_frame_callback_t)(int8_t const frame[LOG_MEL_BIN_COUNT], void * context);

typedef struct {
    int32_t sample_buffer[LOG_MEL_WINDOW_SAMPLES];          /**< hop境界をまたぐPCM */
    float hann_window[LOG_MEL_WINDOW_SAMPLES];              /**< periodic Hann係数 */
    float twiddle_real[LOG_MEL_FFT_SIZE / 2U];              /**< FFT回転因子実部 */
    float twiddle_imag[LOG_MEL_FFT_SIZE / 2U];              /**< FFT回転因子虚部 */
    float mel_edges_hz[LOG_MEL_EDGE_COUNT];                 /**< HTK mel三角フィルタ端点 */
    float fft_real[LOG_MEL_FFT_SIZE];                       /**< FFT作業領域実部 */
    float fft_imag[LOG_MEL_FFT_SIZE];                       /**< FFT作業領域虚部 */
    float power[LOG_MEL_SPECTRUM_BINS];                     /**< 片側power spectrum */
    size_t buffered_samples;                                /**< PCM繰越し数 */
    bool initialized;                                       /**< 係数初期化済み状態 */
} log_mel_extractor_t;                                      /**< 特徴量抽出状態 */

size_t log_mel_extractor_context_size(void);                /* 抽出状態サイズ取得 */
void log_mel_extractor_init(log_mel_extractor_t * extractor); /* 係数とストリーム状態初期化 */
void log_mel_extractor_reset(log_mel_extractor_t * extractor); /* PCM繰越し状態初期化 */
bool log_mel_extractor_self_test(log_mel_extractor_t * extractor); /* 固定ベクトル自己診断 */
void log_mel_extractor_process_window(log_mel_extractor_t * extractor,
                                       int32_t const samples[LOG_MEL_WINDOW_SAMPLES],
                                       int8_t output[LOG_MEL_BIN_COUNT]); /* 1窓の特徴量抽出 */
/* 元のcentered出力を維持し、必要時だけ音量情報を残す絶対log-melも同じFFTで生成する。
 * absolute = clip(round((ln(mel_energy) + 8) / 0.125), -128, 127)。 */
void log_mel_extractor_process_window_pair(log_mel_extractor_t * extractor,
                                           int32_t const samples[LOG_MEL_WINDOW_SAMPLES],
                                           int8_t centered[LOG_MEL_BIN_COUNT],
                                           int8_t absolute[LOG_MEL_BIN_COUNT]);
size_t log_mel_extractor_feed(log_mel_extractor_t * extractor, int32_t const * samples, size_t sample_count,
                              log_mel_frame_callback_t callback, void * context); /* PCMストリーム投入 */

#endif /* RESPEAKER_LOG_MEL_EXTRACTOR_H */
