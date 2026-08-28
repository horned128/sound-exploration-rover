idf_component_register(
    SRCS
        "app_main.c"
        "acoustic_frontend.c"
        "audio_capture.c"
        "usb_link.c"
        "wifi_telemetry.c"
        "xvf3800_control.c"
        "../../common/acoustic_protocol.c"
    INCLUDE_DIRS
        "."
        "../../common"
    REQUIRES
        esp_driver_i2c
        esp_driver_i2s
        esp_timer
        esp_tinyusb
        esp_wifi
        nvs_flash
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
