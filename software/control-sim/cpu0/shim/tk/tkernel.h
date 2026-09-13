#ifndef CPU0_TEST_TKERNEL_H
#define CPU0_TEST_TKERNEL_H
#include "../../../shim/tk/tkernel.h"
#define E_OK 0
#define E_SYS (-5)
#define TA_HLNG 1U
#define TA_RNG3 0x300U
#define TA_TFIFO 0U
#define TWF_ORW 1U
#define TWF_BITCLR 0x20U
#define TMO_FEVR (-1)
#define E_PAR (-17)
#define E_NOEXS (-42)
#define E_TMOUT (-50)
#define TMO_POL 0
#define TA_WSGL 0U
typedef void (*FP)(INT, void *);
typedef unsigned int UINT;
typedef struct { W hi; UW lo; } SYSTIM;
typedef struct { void *exinf; UW tskatr; void (*task)(INT, void *); INT itskpri; UW stksz; void *bufptr; } T_CTSK;
typedef struct { void *exinf; UW flgatr; UW iflgptn; } T_CFLG;
typedef struct { void *exinf; UW cycatr; void (*cychdr)(void *); UW cyctim; UW cycphs; } T_CCYC;
ID tk_cre_tsk(const T_CTSK *);
ID tk_cre_cyc(const T_CCYC *);
ID tk_cre_flg(const T_CFLG *);
ER tk_sta_tsk(ID, INT);
ER tk_sta_cyc(ID);
ER tk_stp_cyc(ID);
ER tk_del_cyc(ID);
ER tk_ter_tsk(ID);
ER tk_del_tsk(ID);
ER tk_del_flg(ID);
ER tk_set_flg(ID, UINT);
ER tk_wai_flg(ID, UINT, UINT, UINT *, INT);
ER tk_get_otm(SYSTIM *);
ER tk_dly_tsk(INT);
void tk_ext_tsk(void);
#endif
