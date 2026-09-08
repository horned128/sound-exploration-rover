/* EK-RA8P1用のCPU1固有μT-Kernelクロック定義。 */
#ifndef _MTKBSP_SYS_SYSDEF_DEPEND_H_
#define _MTKBSP_SYS_SYSDEF_DEPEND_H_

#include <sys/sysdepend/ra_fsp/cpu/ra8p1/sysdef.h>

/* CPU1 Cortex-M33はPLL1P / 4のCPUCLK1、250 MHzで動作する。 */
#define CPUCLK_MHz                   (250)
#define ICLK_MHz                     (250)
#define PCLKA_MHz                    (125)
#define PCLKB_MHz                    (62)
#define PCLKC_MHz                    (125)
#define PCLKD_MHz                    (250)
#define PCLKE_MHz                    (250)

#define SYSCLK                       (CPUCLK_MHz * 1000 * 1000)
#define TMCLK_KHz                    (CPUCLK_MHz * 1000)
#define TMCLK                        (CPUCLK_MHz)

#endif /* _MTKBSP_SYS_SYSDEF_DEPEND_H_ */
