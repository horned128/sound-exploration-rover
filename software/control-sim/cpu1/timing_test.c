#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "drivers/encoder.h"
#include "drivers/bts7960.h"
#include "drivers/servo.h"
#include "services/drive_service.h"
#include "services/actuator_service.h"
#include "ipc/actuator_ipc_server.h"
#include "tasks/task_actuator.h"
#include "config/drive_config.h"
#include "config/actuator_config.h"

static H left_pwm, right_pwm;
static int pwm_calls, stops, disables, servo_sets, pending, pins[4], rx_fault;
static H servo_targets[4];
static actuator_command_t command;
static int create_call, fail_create, fail_start, deleted[3], stopped_cycle;
static void (*task_entry)(INT, void *);
static void (*tick_entry)(void *);
static UD fake_now;
static UW flag_bits;
static UW intervals[32];
static unsigned interval_count, interval_index;
static int fail_clock, clock_calls, clock_fail_at, task_exited, run_on_start, cycle_running;

static fsp_err_t pin_read(void *ctrl, bsp_io_port_pin_t pin, bsp_io_level_t *level) {
    (void)ctrl; *level=pins[pin]; return FSP_SUCCESS;
}
static fsp_err_t irq_open(void *a, const void *b) { (void)a; (void)b; return 0; }
static fsp_err_t irq_enable(void *a) { (void)a; return 0; }
static const ioport_api_t io_api={.pinRead=pin_read};
const ioport_instance_t g_ioport={NULL,&io_api};
static const irq_api_t irq_api={irq_open,irq_enable};
const irq_instance_t g_encoder_left_a_irq={NULL,NULL,&irq_api};
const irq_instance_t g_encoder_left_b_irq={NULL,NULL,&irq_api};
const irq_instance_t g_encoder_right_a_irq={NULL,NULL,&irq_api};
const irq_instance_t g_encoder_right_b_irq={NULL,NULL,&irq_api};
fsp_err_t bts7960_init(void) { left_pwm=right_pwm=0; return 0; }
fsp_err_t bts7960_set_signed_duty(H left,H right) { left_pwm=left;right_pwm=right;pwm_calls++;return 0; }
fsp_err_t bts7960_stop(void) { left_pwm=right_pwm=0;stops++;return 0; }
fsp_err_t servo_init(void) { return 0; }
fsp_err_t servo_set_target_deg(UW i,H target) { assert(i<4);servo_targets[i]=target;servo_sets++;return 0; }
fsp_err_t servo_disable(UW i) { (void)i;disables++;return 0; }
fsp_err_t actuator_ipc_server_init(void) { pending=0;return 0; }
BOOL actuator_ipc_server_take_command(actuator_command_t *out) {
    if (!pending) return FALSE;
    *out=command;pending=0;return TRUE;
}
BOOL actuator_ipc_server_take_rx_fault(void) { int ret=rx_fault;rx_fault=0;return ret; }

ID tk_cre_flg(const T_CFLG *c) { flag_bits=c->iflgptn; return ++create_call==fail_create ? -10 : 1; }
ID tk_cre_cyc(const T_CCYC *c) {
    assert(c->cyctim==1 && c->cycphs==1);tick_entry=c->cychdr;
    return ++create_call==fail_create ? -10 : 2;
}
ID tk_cre_tsk(const T_CTSK *c) { task_entry=c->task; return ++create_call==fail_create ? -10 : 3; }
ER tk_sta_tsk(ID id,INT code) {
    assert(id==3 && code==0);
    if (fail_start==1) return -11;
    if (run_on_start) task_entry(0,NULL);
    return 0;
}
ER tk_sta_cyc(ID id) { assert(id==2);if(fail_start==2)return -12;cycle_running=1;return 0; }
ER tk_stp_cyc(ID id) { assert(id==2);stopped_cycle++;cycle_running=0;return 0; }
ER tk_del_flg(ID id) { assert(id==1);deleted[0]++;return 0; }
ER tk_del_cyc(ID id) { assert(id==2);deleted[1]++;return 0; }
ER tk_del_tsk(ID id) { assert(id==3);deleted[2]++;return 0; }
ER tk_ter_tsk(ID id) { assert(id==3);return 0; }
ER tk_set_flg(ID id,UINT bits) { assert(id==1);flag_bits|=bits;return 0; }
ER tk_wai_flg(ID id,UINT bits,UINT mode,UINT *pattern,INT timeout) {
    assert(id==1 && bits==1 && mode==(TWF_ORW|TWF_BITCLR) && timeout==TMO_FEVR);
    if (interval_index==interval_count) return -20;
    fake_now+=intervals[interval_index++];
    /* Multiple notifications collapse to one wake; no simulated millisecond loop. */
    tick_entry(NULL);tick_entry(NULL);
    *pattern=flag_bits;flag_bits&=~bits;return 0;
}
ER tk_get_otm(SYSTIM *out) {
    if (fail_clock || ++clock_calls==clock_fail_at) return -30;
    out->hi=(W)(fake_now>>32);out->lo=(UW)fake_now;return 0;
}
void tk_ext_tsk(void) { task_exited++; }

static void send(UW seq,H rpm,UB enable,UB emergency) {
    command=(actuator_command_t){0};command.sequence_number=seq;
    command.left_target_rpm=command.right_target_rpm=rpm;
    command.actuator_enable=enable;command.emergency_stop=emergency;pending=1;
}
static void reset_fakes(void) {
    pwm_calls=stops=disables=servo_sets=pending=rx_fault=0;
    memset(pins,0,sizeof(pins));
    memset(servo_targets,0,sizeof(servo_targets));
}

static W expected_rpm_x10(W delta, UW elapsed_ms) {
    return (W) (((long long) delta * 600000LL) /
                 ((long long) WHEEL_ENCODER_COUNTS_PER_REV * elapsed_ms));
}

static void test_encoder(void) {
    reset_fakes();assert(encoder_init()==0);encoder_housekeeping(0);
    for (int i=0;i<50;i++) {
        g_encoder_left_count+=2;g_encoder_right_count-=2;encoder_housekeeping(2);
    }
    assert(g_encoder_sample_count==1 && g_encoder_sample_elapsed_ms==100);
    assert(g_encoder_left_rpm_x10==expected_rpm_x10(100, 100));
    assert(g_encoder_right_rpm_x10==expected_rpm_x10(-100, 100));
    /* A delayed poll divides by its actual window, not by nominal 100 ms. */
    g_encoder_left_count+=900;g_encoder_right_count-=900;encoder_housekeeping(200);
    assert(g_encoder_sample_elapsed_ms==200 &&
           g_encoder_left_rpm_x10==expected_rpm_x10(900, 200));
    assert(g_encoder_right_rpm_x10==expected_rpm_x10(-900, 200));
    encoder_housekeeping(0);assert(g_encoder_sample_count==2);
    encoder_housekeeping(100);assert(g_encoder_left_rpm_x10==0);
    /* Count rollover and ISR increments are defined at both signed boundaries. */
    assert(encoder_init()==0);
    g_encoder_left_count=INT32_MAX-44;g_encoder_right_count=INT32_MIN+44;
    encoder_housekeeping(0);
    g_encoder_left_count=INT32_MIN+45;g_encoder_right_count=INT32_MAX-45;
    encoder_housekeeping(100);
    assert(g_encoder_left_rpm_x10==expected_rpm_x10(90, 100));
    assert(g_encoder_right_rpm_x10==expected_rpm_x10(-90, 100));
    assert(encoder_init()==0);
    g_encoder_left_count=INT32_MAX;g_encoder_right_count=INT32_MIN;
    pins[0]=1;encoder_left_a_irq_callback(NULL);
    pins[2]=1;encoder_right_a_irq_callback(NULL);
    assert(g_encoder_left_count==INT32_MIN && g_encoder_right_count==INT32_MAX);
}

static void test_drive(void) {
    reset_fakes();assert(drive_service_init()==0);
    assert(drive_service_set_target_rpm(120,120)==0);
    drive_service_update(0);assert(g_drive_left_duty_permille==0 && pwm_calls==0);
    drive_service_update(2);assert(g_drive_left_duty_permille==-4 && pwm_calls==0);
    drive_service_update(3);assert(left_pwm==-10 && right_pwm==10 && pwm_calls==1);
    drive_service_update(7);assert(left_pwm==-24 && pwm_calls==2);
    drive_service_update(3);assert(left_pwm==-30 && pwm_calls==3);
    drive_service_update(145);assert(left_pwm==-320 && right_pwm==320);
    assert(drive_service_set_target_rpm(-120,-120)==0);
    drive_service_update(5);assert(left_pwm==-310 && right_pwm==310);
    drive_service_update(UINT32_MAX);assert(left_pwm==320 && right_pwm==-320);
    assert(drive_service_set_target_rpm(0,0)==0);
    assert(left_pwm==0 && right_pwm==0 && stops==1);
}

static void test_watchdog(void) {
    reset_fakes();assert(actuator_service_init()==0);actuator_service_update(0);
    send(1,120,1,0);actuator_service_update(500);
    /* Fresh command cannot retroactively consume 500 ms of ramp time. */
    assert(g_drive_left_duty_permille==0);
    for (int i=0;i<749;i++) actuator_service_update(2);
    assert(left_pwm!=0 && !(g_actuator_service_fault_flags & ACTUATOR_FAULT_COMMAND_TIMEOUT));
    actuator_service_update(1);assert(left_pwm!=0);
    int before=stops;
    actuator_service_update(1);
    assert(left_pwm==0 && right_pwm==0 && stops==before+1);
    assert(g_actuator_service_fault_flags & ACTUATOR_FAULT_COMMAND_TIMEOUT);
    actuator_service_update(UINT32_MAX);actuator_service_update(1);
    assert(left_pwm==0 && stops==before+1);
    send(2,120,1,0);actuator_service_update(10);
    assert(!(g_actuator_service_fault_flags & ACTUATOR_FAULT_COMMAND_TIMEOUT));
    actuator_service_update(160);assert(left_pwm==-320);
    actuator_service_update(2000);assert(left_pwm==0);
    send(3,120,1,0);actuator_service_update(1);actuator_service_update(160);
    send(4,120,1,1);actuator_service_update(1);assert(left_pwm==0);
    send(5,120,1,0);actuator_service_update(1);actuator_service_update(160);
    assert(left_pwm==0 && (g_actuator_service_fault_flags & ACTUATOR_FAULT_EMERGENCY_STOP_ACTIVE));
    send(6,0,0,0);actuator_service_update(1);
    assert(!(g_actuator_service_fault_flags & ACTUATOR_FAULT_EMERGENCY_STOP_ACTIVE));
    send(7,120,1,0);actuator_service_update(1);actuator_service_update(160);assert(left_pwm==-320);
    send(8,0,1,0);actuator_service_update(1);assert(left_pwm==0);
    /* New command wins at timeout boundary, without resetting the emergency latch. */
    send(9,120,1,0);actuator_service_update(1);
    send(10,120,1,0);actuator_service_update(1500);
    assert(!(g_actuator_service_fault_flags & ACTUATOR_FAULT_COMMAND_TIMEOUT));
}

static void test_boot_center(void) {
    reset_fakes();
    assert(actuator_service_init()==0);
    assert(servo_sets==4 && left_pwm==0 && right_pwm==0);
    for (int i=0;i<4;i++) assert(servo_targets[i]==0);

    int const initial_disables=disables;
    send(1,0,0,0);actuator_service_update(999);
    /* LISTENの無効指令では、0度へ戻すためのPWMを途中で止めない。 */
    assert(disables==initial_disables && left_pwm==0 && right_pwm==0);
    actuator_service_update(1);
    assert(disables==initial_disables+4 && left_pwm==0 && right_pwm==0);

    reset_fakes();
    assert(actuator_service_init()==0);
    int const preempt_disables=disables;
    send(2,0,1,0);command.servo_target_deg[0]=15;actuator_service_update(1);
    assert(servo_targets[0]==15);
    actuator_service_update(ACTUATOR_BOOT_CENTER_HOLD_MS);
    /* 有効な走行指令が先に来た場合は、起動整定が後から停止しない。 */
    assert(disables==preempt_disables && left_pwm==0 && right_pwm==0);
}

#if DRIVE_MEASUREMENT_TEST_ENABLE
static void test_measurement_override(void) {
    reset_fakes();
    assert(actuator_service_init()==0);
    actuator_service_update(0);
    assert(g_drive_measurement_status==DRIVE_MEASUREMENT_STATUS_DISABLED);
    assert(!g_drive_measurement_output_authorized && left_pwm==0 && right_pwm==0);

    /* 計測モードだけでは出力せず、有効な通常指令を受けてから許可する。 */
    g_drive_measurement_mode=DRIVE_MEASUREMENT_MODE_DUTY_OVERRIDE;
    g_drive_measurement_duty_permille=320U;
    actuator_service_update(1);
    assert(g_drive_measurement_status==DRIVE_MEASUREMENT_STATUS_WAITING_FOR_ENABLE);
    assert(!g_drive_measurement_output_authorized && left_pwm==0 && right_pwm==0);

    send(1,0,1,0);
    actuator_service_update(1);
    assert(g_drive_measurement_status==DRIVE_MEASUREMENT_STATUS_ACTIVE);
    assert(g_drive_measurement_output_authorized);
    assert(g_drive_measurement_applied_duty_permille==320U);
    actuator_service_update(160);
    assert(left_pwm==-320 && right_pwm==320);

    /* 入力値は上限560‰へ丸める。通常のMAX(700‰)を超えて出せない。 */
    g_drive_measurement_duty_permille=(UH) 0xFFFFU;
    actuator_service_update(1);
    assert(g_drive_measurement_status==DRIVE_MEASUREMENT_STATUS_ACTIVE);
    assert(g_drive_measurement_applied_duty_permille==DRIVE_MEASUREMENT_MAX_DUTY_PERMILLE);

    /* CPU0指令が途切れたら、計測モードでも既存timeoutで停止する。 */
    actuator_service_update(1500);
    assert(left_pwm==0 && right_pwm==0);
    assert(g_drive_measurement_status==DRIVE_MEASUREMENT_STATUS_WAITING_FOR_ENABLE);
    assert(!g_drive_measurement_output_authorized);
    assert(g_actuator_service_fault_flags & ACTUATOR_FAULT_COMMAND_TIMEOUT);

    /* 強制停止は有効指令の有無に関係なく停止を維持する。 */
    g_drive_measurement_mode=DRIVE_MEASUREMENT_MODE_FORCE_STOP;
    actuator_service_update(1);
    assert(g_drive_measurement_status==DRIVE_MEASUREMENT_STATUS_FORCE_STOP);
    assert(left_pwm==0 && right_pwm==0 && !g_drive_measurement_output_authorized);
    assert(g_drive_measurement_stop_capture_valid);

    g_drive_measurement_mode=DRIVE_MEASUREMENT_MODE_NORMAL;
    send(2,0,0,0);
    actuator_service_update(1);
    assert(g_drive_measurement_status==DRIVE_MEASUREMENT_STATUS_DISABLED);
    assert(left_pwm==0 && right_pwm==0);
}
#endif

static void reset_task_fakes(void) {
    create_call=fail_create=fail_start=stopped_cycle=0;
    memset(deleted,0,sizeof(deleted));interval_index=interval_count=0;
    fail_clock=clock_calls=clock_fail_at=task_exited=run_on_start=cycle_running=0;
}
static void test_task(void) {
    reset_task_fakes();assert(task_actuator_create()==0);
    fail_clock=run_on_start=1;assert(task_actuator_start()==0);
    assert(task_exited==1 && cycle_running==0 && left_pwm==0);
    task_actuator_delete();
    for (int fail=1;fail<=3;fail++) {
        reset_task_fakes();fail_create=fail;
        assert(task_actuator_create()==APP_FAULT_ACTUATOR_TASK_CREATE);
        assert(g_task_actuator_last_error==-10);
        assert(deleted[0]==(fail>1) && deleted[1]==(fail>2) && deleted[2]==0);
        task_actuator_delete();
    }
    for (int fail=1;fail<=2;fail++) {
        reset_task_fakes();assert(task_actuator_create()==0);fail_start=fail;
        assert(task_actuator_start()==APP_FAULT_ACTUATOR_TASK_START);
        assert(deleted[0]==1 && deleted[1]==1 && deleted[2]==1);
        assert(left_pwm==0);
    }
    reset_task_fakes();assert(task_actuator_create()==0);assert(task_actuator_start()==0);
    fake_now=UINT32_MAX-1ULL;
    intervals[0]=2;intervals[1]=0;intervals[2]=3;intervals[3]=145;
    interval_count=4;send(11,120,1,0);
    task_entry(0,NULL);
    assert(task_exited==1 && g_task_actuator_last_error==-20);
    assert(g_task_actuator_update_count==3 && g_task_actuator_elapsed_total_ms==150);
    assert(g_task_actuator_period_min_ms==2 && g_task_actuator_period_max_ms==145);
    assert(g_task_actuator_late_count==3 && left_pwm==0 && stopped_cycle==1);
    task_actuator_delete();
    reset_task_fakes();assert(task_actuator_create()==0);
    fail_clock=1;task_entry(0,NULL);
    assert(g_task_actuator_last_error==-30 && task_exited==1 && left_pwm==0);
    task_actuator_delete();
    reset_task_fakes();assert(task_actuator_create()==0);
    interval_count=1;intervals[0]=1;clock_fail_at=2;task_entry(0,NULL);
    assert(g_task_actuator_last_error==-30 && g_task_actuator_update_count==0 && left_pwm==0);
    task_actuator_delete();
}

static void test_feedback_delay(void) {
    reset_fakes();assert(encoder_init()==0);assert(drive_service_init()==0);
    assert(drive_service_set_target_rpm(120,120)==0);
    drive_service_update(149);
    assert(drive_service_set_target_rpm(120,120)==0);
    drive_service_update(1);
    /* Same target at exactly 150 real milliseconds must enable feedback. */
    assert(drive_service_set_target_rpm(120,120)==0);
    drive_service_update(130);
    assert(left_pwm==-560 && right_pwm==560);
    drive_service_stop();
}

int main(void) {
    if (DRIVE_SPEED_FEEDBACK_ENABLE) {
        test_feedback_delay();
    } else {
        test_encoder();test_drive();test_watchdog();test_boot_center();test_task();
    }
#if DRIVE_MEASUREMENT_TEST_ENABLE
    test_measurement_override();
#endif
    puts("CPU1 timing: RPM, jitter, ramp, watchdog, emergency latch, clock rollover and resource cleanup passed");
    return 0;
}
