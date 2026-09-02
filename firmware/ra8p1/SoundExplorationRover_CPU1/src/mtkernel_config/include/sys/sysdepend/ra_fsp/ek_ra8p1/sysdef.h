/* CPU1-specific μT-Kernel clock definition for EK-RA8P1. */
#ifndef _MTKBSP_SYS_SYSDEF_DEPEND_H_
#define _MTKBSP_SYS_SYSDEF_DEPEND_H_

#include <sys/sysdepend/ra_fsp/cpu/ra8p1/sysdef.h>

/* CPU1 Cortex-M33 runs from CPUCLK1 at PLL1P / 4 = 250 MHz. */
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
