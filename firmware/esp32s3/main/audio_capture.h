#ifndef RESPEAKER_AUDIO_CAPTURE_H
#define RESPEAKER_AUDIO_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int16_t level_dbfs_x100;
    int16_t peak_dbfs_x100;
    uint32_t frame_count;
    uint32_t overrun_count;
    uint32_t captured_at_ms;
    bool valid;
} audio_capture_snapshot_t;

esp_err_t audio_capture_start(void);
void audio_capture_get_snapshot(audio_capture_snapshot_t *snapshot);

#endif /* RESPEAKER_AUDIO_CAPTURE_H */
