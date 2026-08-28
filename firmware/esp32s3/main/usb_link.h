#ifndef RESPEAKER_USB_LINK_H
#define RESPEAKER_USB_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t usb_link_init(void);
bool usb_link_is_mounted(void);
bool usb_link_take_new_session(void);
esp_err_t usb_link_send(uint8_t const *data, size_t length);
esp_err_t usb_link_receive(uint8_t *data,
                           size_t capacity,
                           size_t *received_length,
                           uint32_t timeout_ms);
uint32_t usb_link_rx_drop_count(void);

#endif /* RESPEAKER_USB_LINK_H */
