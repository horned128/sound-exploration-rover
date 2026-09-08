/* RA8P1用のCPU1固有μT-Kernelメモリ・割込み定義。 */
#ifndef _MTKBSP_SYS_SYSDEF_DEPEND_CPU_H_
#define _MTKBSP_SYS_SYSDEF_DEPEND_CPU_H_

#include <sys/machine.h>
#include <sys/sysdepend/ra_fsp/cpu/core/armv8m/sysdef.h>

/* FSPのマルチコア配置では、SRAM上位936 KiBをCPU1へ割り当てる。 */
#define INTERNAL_RAM_START           (0x220EA000)
#define INTERNAL_RAM_SIZE            (0x000EA000)
#define INTERNAL_RAM_END             (INTERNAL_RAM_START + INTERNAL_RAM_SIZE)
#define INITIAL_SP                   (INTERNAL_RAM_END)

#define MIN_TIMER_PERIOD             (1)
#define MAX_TIMER_PERIOD             (50)

#define N_SYSVEC                     (16)
#define N_INTVEC                     (96)
#define EXCTBL_ALIGN                 (512)
#define INTPRI_BITWIDTH              (4)

#define INTPRI_MAX_EXTINT_PRI        (1)
#define INTPRI_SVC                   (0)
#define INTPRI_SYSTICK               (1)
#define INTPRI_PENDSV                (15)
#define TIMER_INTLEVEL               (0)

#define CPU_HAS_FPU                  (1)
#define CPU_HAS_DSP                  (0)

#if USE_FPU
#define NUM_COPROCESSOR              (1)
#else
#define NUM_COPROCESSOR              (0)
#endif

#define MTK_PORT0_BASE               (0x40400000)
#define MTK_PORT1_BASE               (0x40400020)
#define MTK_PORT2_BASE               (0x40400040)
#define MTK_PORT3_BASE               (0x40400060)
#define MTK_PORT4_BASE               (0x40400080)
#define MTK_PORT5_BASE               (0x404000A0)
#define MTK_PORT6_BASE               (0x404000C0)
#define MTK_PORT7_BASE               (0x404000E0)
#define MTK_PORT8_BASE               (0x40400100)
#define MTK_PORT9_BASE               (0x40400120)
#define MTK_PORTA_BASE               (0x40400140)
#define MTK_PORTB_BASE               (0x40400160)
#define MTK_PORTC_BASE               (0x40400180)
#define MTK_PORTD_BASE               (0x404001A0)

#define PORT_PODR(n)                 (MTK_PORT##n##_BASE + 0x02)
#define PORT_PIDR(n)                 (MTK_PORT##n##_BASE + 0x06)

#define CPU_HAS_PTMR                 (0)

#endif /* _MTKBSP_SYS_SYSDEF_DEPEND_CPU_H_ */
