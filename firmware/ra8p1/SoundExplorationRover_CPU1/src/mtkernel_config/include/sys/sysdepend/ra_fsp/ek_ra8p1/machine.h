/* EK-RA8P1用のCPU1固有μT-Kernelマシン定義。 */
#ifndef _MTKBSP_SYS_SYSDEPEND_MACHINE_H_
#define _MTKBSP_SYS_SYSDEPEND_MACHINE_H_

#define MTKBSP_RAFSP                 (1)
#define MTKBSP_EK_RA8P1              (1)

#define MTKBSP_CPU_RA                (1)
#define MTKBSP_CPU_RA8               (1)
#define MTKBSP_CPU_RA8P1             (1)

#define MTKBSP_CPU_CORE_ARMV8M       (1)
#define MTKBSP_CPU_CORE_ACM33        (1)

#define KNL_SYSDEP_PATH              sysdepend/ra_fsp
#define TARGET_DIR                   ra_fsp/ek_ra8p1
#define TARGET_GRP_DIR               ra_fsp
#define TARGET_CPU_DIR               ra8p1

#include <sys/sysdepend/ra_fsp/cpu/core/armv8m/machine.h>

#endif /* _MTKBSP_SYS_SYSDEPEND_MACHINE_H_ */
