/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_usb_basic.h"
#include "r_usb_basic_api.h"
#include "r_usb_hcdc_api.h"
#include "r_iic_master.h"
#include "r_i2c_master_api.h"
#include "r_ipc.h"
FSP_HEADER
/* Basic on USB Instance. */
extern const usb_instance_t g_basic0;

/** Access the USB instance using these structures when calling API functions directly (::p_api is not used). */
extern usb_instance_ctrl_t g_basic0_ctrl;
extern const usb_cfg_t g_basic0_cfg;

#ifndef NULL
void NULL(void*);
#endif

#if 0 == BSP_CFG_RTOS
#ifndef NULL
void NULL(usb_callback_args_t*);
#endif
#endif

#if 2 == BSP_CFG_RTOS
#ifndef NULL
void NULL(usb_event_info_t *, usb_hdl_t, usb_onoff_t);
#endif
#endif
/** CDC Driver on USB Instance. */
/* I2C Master on IIC Instance. */
extern const i2c_master_instance_t g_i2c_sensor;

/** Access the I2C Master instance using these structures when calling API functions directly (::p_api is not used). */
extern iic_master_instance_ctrl_t g_i2c_sensor_ctrl;
extern const i2c_master_cfg_t g_i2c_sensor_cfg;

#ifndef NULL
void NULL(i2c_master_callback_args_t *p_args);
#endif
/** IPC Instance. */
extern const ipc_instance_t g_actuator_ipc;

/** Access the IPC instance using these structures when calling API functions directly
 (::p_api is not used). */
extern ipc_instance_ctrl_t g_actuator_ipc_ctrl;
extern const ipc_cfg_t g_actuator_ipc_cfg;

#ifndef actuator_ipc_client_callback
void actuator_ipc_client_callback(ipc_callback_args_t *p_args);
#endif
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
