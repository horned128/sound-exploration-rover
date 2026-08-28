#ifndef RESPEAKER_APP_CONFIG_H
#define RESPEAKER_APP_CONFIG_H

#include "driver/gpio.h"

/* ReSpeaker XVF3800とXIAO ESP32S3の基板内接続。 */
#define APP_XVF_I2C_SDA_GPIO                 GPIO_NUM_5
#define APP_XVF_I2C_SCL_GPIO                 GPIO_NUM_6
#define APP_XVF_I2C_FREQUENCY_HZ             (100000U)
#define APP_XVF_I2S_WS_GPIO                  GPIO_NUM_7
#define APP_XVF_I2S_BCLK_GPIO                GPIO_NUM_8
#define APP_XVF_I2S_DIN_GPIO                 GPIO_NUM_43
#define APP_XVF_I2S_DOUT_GPIO                GPIO_NUM_44

#define APP_AUDIO_SAMPLE_RATE_HZ             (16000U)
#define APP_AUDIO_CHANNEL_COUNT              (2U)
#define APP_AUDIO_BITS_PER_SAMPLE            (32U)
#define APP_AUDIO_BLOCK_FRAMES               (256U)
#define APP_AUDIO_STALE_TIMEOUT_MS           (100U)

#define APP_OBSERVATION_PERIOD_MS            (50U)
#define APP_HEALTH_PERIOD_MS                 (1000U)
#define APP_HELLO_PERIOD_MS                  (1000U)
#define APP_XVF_I2C_TIMEOUT_MS               (100U)
#define APP_USB_TX_TIMEOUT_MS                (20U)

/* 接続先PCと同じLANの値を設定する。空文字の間はWi-Fiを開始しない。 */
#define APP_WIFI_SSID                        "Fight Club"
#define APP_WIFI_PASSWORD                    "soap1999"
#define APP_UDP_DESTINATION_IPV4             "192.168.0.165"
#define APP_UDP_DESTINATION_PORT             (5005U)
#define APP_UDP_TELEMETRY_PERIOD_MS          (250U)
#define APP_TELEMETRY_USB_POLL_MS            (20U)

#define APP_AUDIO_TASK_STACK_SIZE            (6144U)
#define APP_FRONTEND_TASK_STACK_SIZE         (4096U)
#define APP_TELEMETRY_TASK_STACK_SIZE        (5120U)
#define APP_AUDIO_TASK_PRIORITY              (6U)
#define APP_FRONTEND_TASK_PRIORITY           (5U)
#define APP_TELEMETRY_TASK_PRIORITY          (4U)

#endif /* RESPEAKER_APP_CONFIG_H */
