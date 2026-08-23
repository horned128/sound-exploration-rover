/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_gpt.h"
#include "r_timer_api.h"
#include "r_ipc.h"
FSP_HEADER
/** Timer on GPT Instance. */
extern const timer_instance_t g_servo_pwm_rr;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_servo_pwm_rr_ctrl;
extern const timer_cfg_t g_servo_pwm_rr_cfg;

#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_servo_pwm_rl;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_servo_pwm_rl_ctrl;
extern const timer_cfg_t g_servo_pwm_rl_cfg;

#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_servo_pwm_fr;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_servo_pwm_fr_ctrl;
extern const timer_cfg_t g_servo_pwm_fr_cfg;

#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_motor_pwm_lpwm;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_motor_pwm_lpwm_ctrl;
extern const timer_cfg_t g_motor_pwm_lpwm_cfg;

#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_motor_pwm;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_motor_pwm_ctrl;
extern const timer_cfg_t g_motor_pwm_cfg;

#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_servo_pwm_fl;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_servo_pwm_fl_ctrl;
extern const timer_cfg_t g_servo_pwm_fl_cfg;

#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
/** IPC Instance. */
extern const ipc_instance_t g_actuator_ipc;

/** Access the IPC instance using these structures when calling API functions directly
 (::p_api is not used). */
extern ipc_instance_ctrl_t g_actuator_ipc_ctrl;
extern const ipc_cfg_t g_actuator_ipc_cfg;

#ifndef actuator_ipc_callback
void actuator_ipc_callback(ipc_callback_args_t *p_args);
#endif
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
