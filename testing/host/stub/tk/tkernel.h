/* Host stub: just enough of micro T-Kernel's types for the pure logic files. */
#ifndef HOST_STUB_TKERNEL_H
#define HOST_STUB_TKERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef int32_t  INT;   typedef uint32_t UINT;
typedef int32_t  W;     typedef uint32_t UW;
typedef int16_t  H;     typedef uint16_t UH;
typedef int8_t   B;     typedef uint8_t  UB;
typedef int32_t  ID;    typedef int32_t  ER;   typedef int32_t PRI;
typedef int32_t  TMO;   typedef uint32_t RELTIM;
typedef void    (*FP)(void);
typedef struct { W hi; UW lo; } SYSTIM;
typedef int      BOOL;
#define TRUE  1
#define FALSE 0
#define E_OK  0
#define IMPORT extern
#define EXPORT
#define LOCAL  static
#endif
