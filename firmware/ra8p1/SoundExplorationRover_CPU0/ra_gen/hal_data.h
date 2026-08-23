/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_ipc.h"
FSP_HEADER
/** IPC Instance. */
extern const ipc_instance_t g_actuator_ipc;

/** Access the IPC instance using these structures when calling API functions directly
 (::p_api is not used). */
extern ipc_instance_ctrl_t g_actuator_ipc_ctrl;
extern const ipc_cfg_t g_actuator_ipc_cfg;

#ifndef NULL
void NULL(ipc_callback_args_t *p_args);
#endif
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
