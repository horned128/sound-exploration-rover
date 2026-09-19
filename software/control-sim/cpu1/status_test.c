/* 実際の状態タスクを実行し、時刻・RTOS・LED・状態取得・IPC送信の境界を模擬する。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "tasks/task_status.h"
#include "services/actuator_service.h"
#include "ipc/actuator_ipc_server.h"
#include "config/task_config.h"

static void (*entry)(INT, void *);
static void (*tick)(void *);
static UD now, epoch, captures[128], completed[128], busy_from, busy_until;
static UW steps[128], default_step, remaining, step_count, step_index, bits;
static unsigned capture_count, commits, calls, next_word, retries, led_changes;
static UW expected_sequence;
static actuator_status_t snapshot;
static int created, fail_create, fail_start, deleted[3], stopped, running, exited;
static int clock_calls, fail_clock_at, run_on_start, in_tick, reverse_at;
static bsp_io_level_t led;
volatile fsp_err_t g_actuator_service_last_error;
static const bsp_io_port_pin_t led_pins[]={0,1,2};
bsp_leds_t g_bsp_leds={3,led_pins};

static fsp_err_t write_pin(void *ctrl,bsp_io_port_pin_t pin,bsp_io_level_t level) {
    (void)ctrl;(void)pin; assert(!in_tick);led=level;led_changes++;return 0;
}
static const ioport_api_t api={.pinWrite=write_pin};
const ioport_instance_t g_ioport={NULL,&api};
void actuator_service_shutdown(void) { assert(!"status runtime must not shut down actuator service"); }
void actuator_service_status_get(actuator_status_t *out) {
    assert(!in_tick && next_word==0 && capture_count<128);
    captures[capture_count++]=now-epoch;
    out->left_duty_permille=(H)(capture_count%1000);
    snapshot=*out;
}
fsp_err_t actuator_ipc_server_send_status_word(const actuator_status_t *s,UB word) {
    assert(!in_tick && word==next_word && memcmp(s,&snapshot,sizeof(*s))==0);
    assert(s->sequence_number==expected_sequence);
    calls++;
    if (now-epoch>=busy_from && now-epoch<busy_until) { retries++;return 1; }
    next_word++;
    if (next_word==ACTUATOR_IPC_STATUS_WORD_COUNT) {
        completed[commits++]=now-epoch;next_word=0;
        expected_sequence=(expected_sequence+1)&ACTUATOR_IPC_SEQUENCE_MASK;
    }
    return 0;
}
ID tk_cre_flg(const T_CFLG *c) { bits=c->iflgptn;return ++created==fail_create?-10:1; }
ID tk_cre_cyc(const T_CCYC *c) {
    assert(c->cyctim==10 && c->cycphs==10);tick=c->cychdr;
    return ++created==fail_create?-10:2;
}
ID tk_cre_tsk(const T_CTSK *c) { entry=c->task;return ++created==fail_create?-10:3; }
ER tk_sta_cyc(ID id) { assert(id==2);if(fail_start==2)return -12;running=1;return 0; }
ER tk_sta_tsk(ID id,INT code) {
    assert(id==3 && code==0);if(fail_start==1)return -11;
    if(run_on_start)entry(0,NULL);return 0;
}
ER tk_stp_cyc(ID id) { assert(id==2);running=0;stopped++;return 0; }
ER tk_del_flg(ID id) { assert(id==1);deleted[0]++;return 0; }
ER tk_del_cyc(ID id) { assert(id==2);deleted[1]++;return 0; }
ER tk_del_tsk(ID id) { assert(id==3);deleted[2]++;return 0; }
ER tk_ter_tsk(ID id) { assert(id==3);return 0; }
ER tk_set_flg(ID id,UINT value) { assert(id==1);bits|=value;return 0; }
ER tk_wai_flg(ID id,UINT value,UINT mode,UINT *pattern,INT timeout) {
    assert(id==1 && value==1 && mode==(TWF_ORW|TWF_BITCLR) && timeout==TMO_FEVR);
    if (!remaining) return -20;
    remaining--;
    now+=step_index<step_count?steps[step_index++]:default_step;
    in_tick=1;tick(NULL);tick(NULL);in_tick=0;
    *pattern=bits;bits&=~value;return 0;
}
ER tk_get_otm(SYSTIM *out) {
    clock_calls++;
    if(clock_calls==fail_clock_at)return -30;
    if(clock_calls==reverse_at)now=epoch-1;
    out->hi=(W)(now>>32);out->lo=(UW)now;return 0;
}
ER tk_dly_tsk(INT delay) { (void)delay;assert(!"relative wait in status loop");return 0; }
void tk_ext_tsk(void) { exited++; }

static void reset(void) {
    task_status_delete();
    memset(deleted,0,sizeof(deleted));
    created=fail_create=fail_start=stopped=running=exited=clock_calls=fail_clock_at=0;
    run_on_start=in_tick=reverse_at=0;
    capture_count=commits=calls=next_word=retries=led_changes=expected_sequence=0;
    now=epoch=busy_from=busy_until=0;
    default_step=10;remaining=step_count=step_index=bits=0;
    g_actuator_service_last_error=0;led=BSP_IO_LEVEL_HIGH;
}
static void run(UW count) {
    assert(task_status_create()==APP_FAULT_NONE);
    assert(task_status_start()==APP_FAULT_NONE);
    remaining=count;entry(0,NULL);
    assert(exited==1 && !running && g_task_status_last_error==-20);
    assert(g_task_status_send_retry_count==retries);
    assert(g_task_status_snapshot_count==capture_count && g_task_status_telemetry_count==commits);
}
static void test_schedule(void) {
    reset();steps[0]=11;step_count=1;run(100);
    assert(g_task_status_update_count==100 && g_task_status_elapsed_total_ms==1001);
    assert(g_task_status_period_min_ms==10 && g_task_status_period_max_ms==11);
    assert(g_task_status_late_count==1 && g_task_status_period_last_ms==10);
    assert(capture_count==10 && commits==10 && calls==100);
    for(unsigned i=0;i<capture_count;i++)assert(captures[i]==101+100*i);
    for(unsigned i=0;i<commits;i++)assert(completed[i]==101+100*i);
    assert(g_task_status_snapshot_period_last_ms==100);
    /* Two heartbeat transitions and the exit's LED-off write. */
    assert(led_changes==3 && led==BSP_IO_LEVEL_HIGH);
}
static void test_busy_and_delay(void) {
    reset();busy_from=120;busy_until=350;run(50);
    assert(capture_count==5 && commits==5 && retries==15);
    assert(captures[0]==100 && captures[1]==200 && captures[2]==360 && captures[3]==400 && captures[4]==500);
    assert(completed[0]==100 && completed[1]==350 && completed[2]==360 && completed[3]==400 && completed[4]==500);
    /* A missed interval recovers immediately on wake, keeping subsequent frames. */
    reset();steps[0]=100;steps[1]=550;steps[2]=0;step_count=3;run(10);
    assert(g_task_status_update_count==9 && g_task_status_elapsed_total_ms==720);
    assert(g_task_status_period_max_ms==550 && capture_count==3 && commits==3 && calls==30);
    assert(captures[0]==100 && captures[1]==650 && captures[2]==700);
    assert(completed[0]==100 && completed[1]==650 && completed[2]==700);
    assert(g_task_status_snapshot_period_last_ms==50);
}
static void test_time_wrap(void) {
    reset();epoch=now=(UD)UINT32_MAX-50;run(20);
    assert(g_task_status_elapsed_total_ms==200 && captures[0]==100 && captures[1]==200);
    reset();steps[0]=UINT32_MAX;steps[1]=UINT32_MAX;step_count=2;run(2);
    assert(g_task_status_elapsed_total_ms==(UD)UINT32_MAX*2 && calls==20 && capture_count==2);
}
static void test_failures(void) {
    for(int n=1;n<=3;n++) {
        reset();fail_create=n;assert(task_status_create()==APP_FAULT_STATUS_TASK_CREATE);
        assert(g_task_status_last_error==-10 && deleted[0]==(n>1) && deleted[1]==(n>2));
        task_status_delete();assert(deleted[0]==(n>1) && deleted[1]==(n>2));
    }
    for(int n=1;n<=2;n++) {
        reset();assert(task_status_create()==0);fail_start=n;
        assert(task_status_start()==APP_FAULT_STATUS_TASK_START && !running);
        assert(deleted[0]==1 && deleted[1]==1 && deleted[2]==1);
    }
    for(int n=1;n<=2;n++) {
        reset();assert(task_status_create()==0);fail_clock_at=n;remaining=1;run_on_start=1;
        assert(task_status_start()==0 && exited==1 && !running);
        assert(g_task_status_last_error==-30 && !capture_count);
    }
    reset();assert(task_status_create()==0);now=epoch=100;reverse_at=2;remaining=1;
    assert(task_status_start()==0);entry(0,NULL);
    assert(g_task_status_last_error==E_SYS && exited==1 && !running && !capture_count);
    reset();g_actuator_service_last_error=1;run(10);
    assert(led_changes==3); /* 50 ms, 100 ms, exit. */
}
int main(void) {
    test_schedule();test_busy_and_delay();test_time_wrap();test_failures();
    puts("CPU1 status: schedule, packet integrity, delayed/busy IPC, wrap and failure paths passed");
    return 0;
}
