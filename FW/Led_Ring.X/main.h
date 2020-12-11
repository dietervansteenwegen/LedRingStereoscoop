/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef MAIN_H
#define	MAIN_H

#define FCY 8000000UL    // Instruction cycle frequency, Hz - required for __delayXXX() to work
#include <xc.h> // include processor files - each processor file is guarded.  

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "hardware.h"
#include "uart.h"
#include <libpic30.h>           // has __delay_ms() function

/* METHODS */

/* GENERIC STUFF */
#define    HIGH             1
#define    LOW              0
#define    SET              1
#define    CLEAR            0

/* Specify an extension for GCC based compilers */
#if defined(__GNUC__)
#define __EXTENSION __extension__
#else
#define __EXTENSION
#endif

#if !defined(__PACKED)
    #define __PACKED
#endif

typedef union
{
    BYTE Val;
    struct __PACKED
    {
        __EXTENSION BYTE b0:1;
        __EXTENSION BYTE b1:1;
        __EXTENSION BYTE b2:1;
        __EXTENSION BYTE b3:1;
        __EXTENSION BYTE b4:1;
        __EXTENSION BYTE b5:1;
        __EXTENSION BYTE b6:1;
        __EXTENSION BYTE b7:1;
    } bits;
} BYTE_VAL, BYTE_BITS;

void start_routine (void);
bool check_inputs (void);
void setup (void);

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

