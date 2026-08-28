#ifndef RESPEAKER_WIFI_TELEMETRY_H
#define RESPEAKER_WIFI_TELEMETRY_H

#include <stdbool.h>
#include "esp_err.h"

esp_err_t wifi_telemetry_start(void);
bool wifi_telemetry_is_connected(void);

#endif /* RESPEAKER_WIFI_TELEMETRY_H */
