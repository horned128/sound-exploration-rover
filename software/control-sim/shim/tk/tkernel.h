#ifndef CONTROL_SIM_TKERNEL_H
#define CONTROL_SIM_TKERNEL_H

#include <stddef.h>
#include <stdint.h>

typedef int8_t B;
typedef uint8_t UB;
typedef int16_t H;
typedef uint16_t UH;
typedef int32_t W;
typedef uint32_t UW;
typedef int64_t D;
typedef uint64_t UD;
typedef int32_t ER;
typedef int32_t ID;
typedef int32_t INT;
typedef int32_t BOOL;

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#ifndef LOCAL
#define LOCAL static
#endif

#ifndef EXPORT
#define EXPORT
#endif

#ifndef IMPORT
#define IMPORT extern
#endif

#ifndef Inline
#define Inline static inline
#endif

#endif /* CONTROL_SIM_TKERNEL_H */

