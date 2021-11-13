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
#include "uart.h"

/***********/
/* METHODS */
/***********/
void init_hardware( void ); 
void initPins( void );
void initPPS( void );
void initAdc (void);
void initOscillator( void );
void initInterrupts (void);
void InitU1(void);
void initTimer1( void );
void initTimer2( void );
uint16_t sampleBatt( void );
void led_start_routine (void);

/***********/
/* DEFINES */
/***********/

/* General defines */
#define    INPUT_PIN        1
#define    OUTPUT_PIN       0
#define    DIGITAL          0
#define    ANALOG           1

/* GPIO */
#define RotPush             PORTAbits.RA7       // Pushbutton on the rotary encoder
#define RotPush_SetDigIn()  _TRISA7=INPUT_PIN           // Set as input
#define RotA                PORTBbits.RB14
#define RotA_SetDigIn()     _TRISB14 = IOUTPUT_PINNPUT_PIN
#define RotA_SetDig()       _ANSB14 = DIGITAL
#define RotB                PORTAbits.RA10
#define RotB_SetDigIn()     _TRISA10 = INPUT_PIN
#define Charging            PORTCbits.RC6
#define Charging_SetDigIn() _TRISC6 = INPUT_PIN
#define Charging_SetDig()   _ANSC6 = DIGITAL


/* UART */
#define U1_Tx_SetDigOut()           _TRISB10=0
#define U1_Tx_SetHighOut()          _LATB10=1
#define U1_Rx_SetDigIn()            _TRISB11=1

/* TP */
#define TP1_SetHigh()          _LATA8 = 1
#define TP1_SetLow()           _LATA8 = 0
#define TP1_Toggle()           _LATA8 ^= 1
#define TP1_SetDigOut()        _TRISA8 = OUTPUT_PIN

/* LEDS */
#define LED_Green                       _LATC3
#define LED_Green_SetDigOut()           _TRISC3 = OUTPUT_PIN
#define LED_Green_SetHigh()             _LATC3 = 1
#define LED_Green_SetLow()              _LATC3 = 0
#define LED_Green_Toggle()              _LATC3 ^= 1

#define LED_Red                       _LATA9
#define LED_Red_SetDigOut()           _TRISA9 = OUTPUT_PIN
#define LED_Red_SetHigh()             _LATA9 = 1
#define LED_Red_SetLow()              _LATA9 = 0
#define LED_Red_Toggle()            _LATA9 ^= 1

#define LED_Heartbeat                    _LATC4
#define LED_Heartbeat_SetHigh()          _LATC4 = 1
#define LED_Heartbeat_SetLow()           _LATC4 = 0
#define LED_Heartbeat_Toggle()           _LATC4 ^= 1
#define LED_Heartbeat_SetDigOut()        _TRISC4 = OUTPUT_PIN

/* Others */
#define LED_Drv_RST                     _LATC6
#define LED_DRV_RST_SetHigh()           _LATC6 = 1
#define LED_DRV_RST_SetLow()            _LATC6 = 0
#define LED_DRV_RST_SetDig_Out()        _TRISC6 = OUTPUT_PIN

#define SHTDWN_BTN                      _PORTBbits.RB2
#define SHTDWN_BTN_SetDig()             _ANSB2 = DIGITAL
#define SHTDWN_BTN_SetDigIn()          _TRISB2 = INPUT_PIN

#define FET_Gate                        _LATA10
#define FET_Gate_LOW                    _LATA10 = 0
#define FET_Gate_HIGH                   _LATA10 = 1
#define FET_Gate_SetDigOut()            _TRISA10 = OUTPUT_PIN

#define StartWDT()              RCONbits.SWDTEN = 0x01;    // Start Software WDT (512ms for PRE = POSTscaler = 128)


#endif	/* XC_HEADER_TEMPLATE_H */

