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

#include <xc.h> // include processor files - each processor file is guarded.  

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "hardware.h"
#include "uart.h"
#define FCY 6000000UL    // Instruction cycle frequency, Hz - required for __delayXXX() to work
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

#define FCY 6000000UL    // Instruction cycle frequency, Hz - required for __delayXXX() to work

typedef struct{
    uint8_t center; // center of the "strip"
    uint8_t width; // width of the strip (max 16 leds)
    uint8_t masterBrightness;    // master brightness of the ring (max 0xFF)
    uint8_t ledBrightness[16];  // brightness of individual leds
    unsigned hasBeenUpdated:1;  // for use with the next stripDef
}structStripDef;

#define masterBright StripDef.masterBrightness
#define updated StripDef.hasBeenUpdated

void updateMasterBright (int8_t direction);
void rotated (int8_t direction);
void stateMachineAdvance ( void );
bool check_inputs (void);
void setup (void);
void setAllLedsManual ( uint8_t );
void initialFillStruct ( structStripDef *);
void updateStrip (void);
#endif	/* XC_HEADER_TEMPLATE_H */

