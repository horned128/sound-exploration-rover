#ifndef CPU1_TEST_HAL_H
#define CPU1_TEST_HAL_H
#include <stdint.h>
#include <tk/tkernel.h>
typedef int fsp_err_t;
#define FSP_SUCCESS 0
#define FSP_ERR_INVALID_ARGUMENT 1
#define FSP_PARAMETER_NOT_USED(x) ((void)(x))
typedef int bsp_io_port_pin_t;
typedef int bsp_io_level_t;
#define BSP_IO_LEVEL_LOW 0
#define BSP_IO_LEVEL_HIGH 1
#define ARDUINO_D2_INT0 0
#define ARDUINO_D1TX_MIKROBUS_TX 1
#define PMOD1_IRQ 2
#define PMOD1_GPIO2 3
typedef struct { int unused; } external_irq_callback_args_t;
typedef struct { int unused; } ipc_callback_args_t;
typedef struct {
    fsp_err_t (*pinRead)(void *, bsp_io_port_pin_t, bsp_io_level_t *);
    fsp_err_t (*pinWrite)(void *, bsp_io_port_pin_t, bsp_io_level_t);
} ioport_api_t;
typedef struct { uint16_t led_count; const bsp_io_port_pin_t *p_leds; } bsp_leds_t;
typedef struct { void *p_ctrl; const ioport_api_t *p_api; } ioport_instance_t;
typedef struct { fsp_err_t (*open)(void *, const void *); fsp_err_t (*enable)(void *); } irq_api_t;
typedef struct { void *p_ctrl; const void *p_cfg; const irq_api_t *p_api; } irq_instance_t;
extern const ioport_instance_t g_ioport;
extern const irq_instance_t g_encoder_left_a_irq, g_encoder_left_b_irq, g_encoder_right_a_irq, g_encoder_right_b_irq;
#endif
