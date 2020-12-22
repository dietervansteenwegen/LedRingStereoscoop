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
#include <stdbool.h>

//extern volatile bool  RotA_GoneHigh, RotA_GoneLow, RotB_GoneHigh, RotB_GoneLow;
extern volatile bool Flag100ms;
extern volatile bool Flag2ms;  // Flag to run the debouncing routine


#endif	/* MAIN_GLOBALS_H */

