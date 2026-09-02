/** =================================================================*
 * @file   sound_follow_controller.h
 * @brief  音源追従行動生成
 * ================================================================= */
#ifndef SEROV_CPU0_SOUND_FOLLOW_CONTROLLER_H
#define SEROV_CPU0_SOUND_FOLLOW_CONTROLLER_H

#include "../../../../../common/acoustic_protocol.h"        /* 音響観測型 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

typedef enum e_cpu0_think_state {
    CPU0_THINK_STATE_WAIT_LINK = 0,
    CPU0_THINK_STATE_LISTEN,
    CPU0_THINK_STATE_STEER_PREP,
    CPU0_THINK_STATE_MOVE_STEP,
    CPU0_THINK_STATE_SETTLE,
    CPU0_THINK_STATE_COOLDOWN,
    CPU0_THINK_STATE_FAULT,
} cpu0_think_state_t;

typedef struct st_sound_follow_input {
    BOOL link_ready;
    BOOL new_observation;
    BOOL fault_active;
    acoustic_observation_t observation;
} sound_follow_input_t;

typedef struct st_sound_follow_output {
    cpu0_think_state_t state;
    H steering_deg;
    H left_rpm;
    H right_rpm;
    BOOL actuator_enable;
    BOOL emergency_stop;
} sound_follow_output_t;

EXPORT void sound_follow_controller_init(void);                    /* 追従状態初期化 */
/* 追従状態更新 */
EXPORT void sound_follow_controller_step(const sound_follow_input_t * p_input,
                                  UW elapsed_ms,
                                  sound_follow_output_t * p_output);

#endif /* SEROV_CPU0_SOUND_FOLLOW_CONTROLLER_H */
