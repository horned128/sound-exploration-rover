/* generated HAL source file - do not edit */
#include "hal_data.h"
ipc_instance_ctrl_t g_actuator_ipc_ctrl;

/** IPC configuration */
const ipc_cfg_t g_actuator_ipc_cfg = { .channel = 0, .p_callback = NULL,
#if defined(NULL)
                .p_context = NULL,
#else
		.p_context = (void*) &NULL,
#endif
		.ipl = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_IPC_IRQ0)
                .irq = VECTOR_NUMBER_IPC_IRQ0,
#else
		.irq = FSP_INVALID_VECTOR,
#endif
		};

/* Instance structure to use this module. */
const ipc_instance_t g_actuator_ipc = { .p_ctrl = &g_actuator_ipc_ctrl, .p_cfg =
		&g_actuator_ipc_cfg, .p_api = &g_ipc_on_ipc };
void g_hal_init(void) {
	g_common_init();
}
