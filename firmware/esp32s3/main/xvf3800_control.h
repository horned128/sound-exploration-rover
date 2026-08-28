#ifndef RESPEAKER_XVF3800_CONTROL_H
#define RESPEAKER_XVF3800_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint16_t doa_deg;
    uint16_t speech_detected_raw;
    uint8_t raw_status;
    bool doa_valid;
    bool used_aec_fallback;
} xvf3800_doa_result_t;

esp_err_t xvf3800_control_init(void);
esp_err_t xvf3800_control_read_doa(xvf3800_doa_result_t *result);

#endif /* RESPEAKER_XVF3800_CONTROL_H */
