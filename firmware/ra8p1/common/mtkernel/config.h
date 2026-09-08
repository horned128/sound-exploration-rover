/*
 * RA8P1両コアで共有するμT-Kernel 3.0設定。
 * プロジェクト固有の設定として、BSP標準設定より優先して読み込む。
 */
#ifndef SEROV_MTKERNEL_CONFIG_H
#define SEROV_MTKERNEL_CONFIG_H

#define CNF_SYSTEMAREA_TOP      (0)
#define CNF_SYSTEMAREA_END      (0)
#define CNF_MAX_TSKPRI          (32)

/* アプリケーションの待ち時間をミリ秒単位で扱う。 */
#define CNF_TIMER_PERIOD        (1)

#define CNF_MAX_TSKID           (32)
#define CNF_MAX_SEMID           (16)
#define CNF_MAX_FLGID           (16)
#define CNF_MAX_MBXID           (8)
#define CNF_MAX_MTXID           (4)
#define CNF_MAX_MBFID           (8)
#define CNF_MAX_MPLID           (4)
#define CNF_MAX_MPFID           (8)
#define CNF_MAX_CYCID           (4)
#define CNF_MAX_ALMID           (8)

#define CNF_MAX_REGDEV          (8)
#define CNF_MAX_OPNDEV          (16)
#define CNF_MAX_REQDEV          (16)
#define CNF_DEVT_MBFSZ0         (-1)
#define CNF_DEVT_MBFSZ1         (-1)

#define CNF_VER_MAKER           (0)
#define CNF_VER_PRID            (0)
#define CNF_VER_PRVER           (3)
#define CNF_VER_PRNO1           (0)
#define CNF_VER_PRNO2           (0)
#define CNF_VER_PRNO3           (0)
#define CNF_VER_PRNO4           (0)

#define USE_LEGACY_API          (0)
#define CNF_MAX_PORID           (0)

#define CNF_EXC_STACK_SIZE      (0)
#define CNF_TMP_STACK_SIZE      (256)

#define USE_NOINIT              (0)
#define USE_IMALLOC             (1)
#define USE_SHUTDOWN            (1)
#define USE_STATIC_IVT          (0)

#define CHK_NOSPT               (1)
#define CHK_RSATR               (1)
#define CHK_PAR                 (1)
#define CHK_ID                  (1)
#define CHK_OACV                (1)
#define CHK_CTX                 (1)
#define CHK_CTX1                (1)
#define CHK_CTX2                (1)
#define CHK_SELF                (1)
#define CHK_TKERNEL_CONST       (1)

#define USE_USERINIT            (0)
#define RI_USERINIT             (0)

#define USE_DBGSPT              (0)
#define USE_OBJECT_NAME         (0)
#define OBJECT_NAME_LENGTH      (8)

#if (_RA_ORDINAL == 2)
/* CPU1はBSPの単一SCI8 T-Monitor端子をCPU0と競合して使用しない。 */
#define USE_TMONITOR            (0)
#define USE_SYSTEM_MESSAGE      (0)
#define USE_EXCEPTION_DBG_MSG   (0)
#else
#define USE_TMONITOR            (1)
#define USE_SYSTEM_MESSAGE      (1)
#define USE_EXCEPTION_DBG_MSG   (1)
#endif
#define USE_TASK_DBG_MSG        (0)

#define USE_FPU                 (1)
#define USE_DSP                 (0)
#define ALWAYS_FPU_ATR          (1)

#define USE_PTMR                (0)
#define USE_SDEV_DRV            (0)
#define USE_STDINC_STDDEF       (1)
#define USE_STDINC_STDINT       (1)

#include "config_bsp.h"
#include <mtkernel/config/config_func.h>

#endif /* SEROV_MTKERNEL_CONFIG_H */
