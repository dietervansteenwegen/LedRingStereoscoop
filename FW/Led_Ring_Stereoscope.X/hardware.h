/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef HARDWARE_H
#define	HARDWARE_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdbool.h>
#include <stdint.h>
#include "uart.h"

/***********/
/* METHODS */
/***********/
void initHardware( void ); 
void initPPS( void );
void initOscillator( void );
void initInterrupts (void);
void InitU1(void);
void initTimer1( void );

/***********/
/* DEFINES */
/***********/
#define    INPUT_PIN        1
#define    OUTPUT_PIN       0
#define    DIGITAL          0
#define    ANALOG           1

/* UART */
#define U1_Tx_SetDigOut()           _TRISB10=0
#define U1_Tx_SetHighOut()          _LATB10=1
#define U1_Rx_SetDigIn()            _TRISB11=1

/* TP */
#define TP1_SetHigh()          _LATB8 = 1
#define TP1_SetLow()           _LATB8 = 0
#define TP1_Toggle()           _LATB8 ^= 1
//#define TP1_SetDig()           _ANSB8 = 0
#define TP1_SetDigOut()        _TRISB8 = 0

/* LEDS */
#define LED_Green_SetDigOut()           _TRISB7 = 0
#define LED_Green                       _LATB7
#define LED_Green_SetHigh()             _LATB7 = 1
#define LED_Green_SetLow()              _LATB7 = 0
#define LED_Green_Toggle()              _LATB7 ^= 1

#define LED_Red_SetDigOut()           _TRISB6 = 0
#define LED_Red                       _LATB6
#define LED_Red_SetHigh()             _LATB6 = 1
#define LED_Red_SetLow()              _LATB6 = 0
#define LED_Red_Toggle()              _LATB6 ^= 1

#define LED_Heartbeat                    _LATB13
#define LED_Heartbeat_SetHigh()          _LATB13 = 1
#define LED_Heartbeat_SetLow()           _LATB13 = 0
#define LED_Heartbeat_Toggle()           _LATB13 ^= 1
#define LED_Heartbeat_SetDigOut()        _TRISB13 = 0

#define StartWDT()                  RCONbits.SWDTEN = 0x01;    // Start Software WDT (512ms for PRE = POSTscaler = 128)


typedef unsigned int            UINT;
typedef unsigned char           UINT8;
typedef unsigned short int      UINT16;
typedef unsigned char           BYTE;                           /* 8-bit unsigned  */
typedef unsigned short int      WORD;                           /* 16-bit unsigned */
typedef unsigned long           DWORD;                          /* 32-bit unsigned */

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

