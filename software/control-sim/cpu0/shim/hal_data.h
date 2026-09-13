#ifndef CPU0_TEST_HAL_H
#define CPU0_TEST_HAL_H
typedef int usb_status_t;
#include "../../cpu1/shim/hal_data.h"
void R_BSP_PinAccessEnable(void);
void R_BSP_PinAccessDisable(void);
void R_BSP_PinWrite(bsp_io_port_pin_t pin, bsp_io_level_t level);

#endif
