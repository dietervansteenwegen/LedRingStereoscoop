/* 
 * File:   main_globals.h
 * Author: dieterv
 * 
 * Contains the variables that are shared by main.c and interrupts.c
 *
 * Created on June 15, 2020, 3:30 PM
 */

#ifndef MAIN_GLOBALS_H
#define	MAIN_GLOBALS_H

#include <stdint.h>

typedef struct{
    unsigned Active:1;              // Did the user switch this output on?
    uint8_t MaxCurrent;             // maximum current for output
}StructUserSettings;

typedef struct{
    unsigned Status:1;         // Last known AlertStatus
    unsigned Changed:1;        // Has AlertStatus been changed, but not handled?
    uint8_t RetriesDone;        // How many retries have been done
    uint16_t msSinceLastRetry;   // how much time has passed after last retry?
}StructAlert;

typedef struct{
    uint8_t Current;            // Last measured current
    uint8_t Voltage;            // Last measured voltage
    unsigned UpdatedUI:1;           // Current and voltage have been updated
    unsigned InternalStatus:1;  // Should output be on (according to us)
}StructStatus;

typedef struct{
    unsigned SWChannel:1;       // SW has switched output off due to overcurrent on this channel
    unsigned SWTotal:1;         // SW has switched output off due to total overcurrent 
    uint16_t msSinceLastRetry;       // how much time has passed since last retry?
    uint8_t OverCurrentRetries;     // Counter for number of retries    
}StructOverCurrentTripped;

typedef struct{
    /* Holds the status of an output, with all required variables*/
    StructUserSettings UserSettings;
    StructAlert Alert;
    StructStatus Status;
    StructOverCurrentTripped OverCurrentTripped;
}OutputStruct;

extern volatile OutputStruct Output1;
extern volatile OutputStruct Output2;
extern volatile OutputStruct Output3;
extern volatile OutputStruct Output4;
extern volatile OutputStruct Output5;
extern volatile OutputStruct Output6;
extern volatile OutputStruct Output7;
extern volatile OutputStruct Output8;


#endif	/* MAIN_GLOBALS_H */

