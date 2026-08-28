/** =================================================================*
 * @file   sound_follow_controller.h
 * @brief  音源追従行動生成
 * ================================================================= */
#ifndef SEROV_CPU0_SOUND_FOLLOW_CONTROLLER_H
#define SEROV_CPU0_SOUND_FOLLOW_CONTROLLER_H

#include <stdbool.h>                                       /* 真偽値 */
#include <stdint.h>                                        /* 固定幅整数型 */
#include "../../../../../common/acoustic_protocol.h"      /* 音響観測型 */

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
    bool link_ready;
    bool new_observation;
    bool fault_active;
    acoustic_observation_t observation;
} sound_follow_input_t;

typedef struct st_sound_follow_output {
    cpu0_think_state_t state;
    int16_t steering_deg;
    int16_t left_rpm;
    int16_t right_rpm;
    bool actuator_enable;
    bool emergency_stop;
} sound_follow_output_t;

void sound_follow_controller_init(void);                   /* 追従状態初期化 */
void sound_follow_controller_step(
    const sound_follow_input_t * p_input,
    uint32_t elapsed_ms,
    sound_follow_output_t * p_output);                     /* 追従状態更新 */

#endif /* SEROV_CPU0_SOUND_FOLLOW_CONTROLLER_H */
