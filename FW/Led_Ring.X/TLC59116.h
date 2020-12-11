/*
 * File:   Sensirion_SHT31.h
 * Author: dieterv
 * v0.2
 * Created on February 3, 2020, 3:09 PM
 * 
 * Library for Sensirion SHT31 Temp/Relative humidity sensor
 * Usage:
 * - Set I2C address in Sensirion_SHT31.c file (const uint8_t SHT31_Address = 0x45)
 *      ! Default address with address pin tied to ground is 0x44
 * - Run SHT31_InitReset
 * - Run SHT31_SingleShot with pointers to where results should be stored and required repeatability
 * - Internal heater can be turned on/off with SHT31_HeaterOnOff: 1 to switch on, 0 to switch off
 */

#ifndef TLC59116
#define	TLC59116
#include <xc.h> // include processor files - each processor file is guarded.  
//#include "I2C1.h"

//--------------------Variables--------------------
//-------------------Macros-------------------

//--------------------Functions--------------------

int8_t TLC59116_InitReset (void);                             // high level
uint8_t setLed(uint8_t led, uint8_t brightness);
uint8_t setAllLeds(uint8_t * brightness1);
#endif

