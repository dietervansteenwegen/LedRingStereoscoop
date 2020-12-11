/*
 * File:   Sensirion_SHT31.c
 * Author: dieterv
 * v0.2
 * Created on February 3, 2020, 3:09 PM
 */

#include <xc.h>
#define    FCY    6000000UL    // Instruction cycle frequency, Hz - required for __delayXXX() to work
#include <libpic30.h>          // For delay functions
#include "TLC59116.h"
#include "stdint.h"
#include "I2C1.h"

// Constants (address and commands):
const uint8_t TLC59116_Address = 0x6F;
// constants with commands are in format:
// {int number of bytes, byte1, byte2}
const uint8_t TLC59116_Mode2_Clear[3] = {2, 0x01, 0x80};
const uint8_t TLC59116_Mode2_Set[3] = {2, 0x01, 0x00};
const uint8_t TLC59116_Mode1[3] = {2, 0x00, 0x01};
const uint8_t TLC59116_GRP_Duty_Cycle[3] = {2, 0x12, 0xFF};
uint8_t TLC59116_PWMx[3] = {2, 0x02, 0x00};
const uint8_t TLC59116_LEDOut0[3] = {2, 0x14, 0xFF};
const uint8_t TLC59116_LEDOut1[3] = {2, 0x15, 0xFF};
const uint8_t TLC59116_LEDOut2[3] = {2, 0x16, 0xFF};
const uint8_t TLC59116_LEDOut3[3] = {2, 0x17, 0xFF};
const uint8_t TLC59116_IRef[3] = {2, 0x17, 0xFF};

uint8_t setLed(uint8_t led, uint8_t brightness){
    if ((led < 1) || (led > 16)){
        return 0;
    }
    if ((brightness < 0) || (brightness > 0xFF)){
        return 0;
    }
    
    TLC59116_PWMx[1] = led + 1;
    TLC59116_PWMx[2] = brightness;
    
    I2C1_Write(  TLC59116_Address,\
                    0xFF,\
                    (uint8_t *)&TLC59116_PWMx[1],\
                    TLC59116_PWMx[0]\
                    );
    return 1;
}

uint8_t setAllLeds(uint8_t * brightness1){
    // sets led 1-16 brightness from array starting at * brightness
    // uses the auto-increment flag to send all brightness values sequentially without 
    // the need for individual adressing.
    // expects 16 values starting at *brightness1
    I2C1_Write(TLC59116_Address, 0xFF, brightness1, 16);
    return 1;
}

int8_t TLC59116_InitReset(void){
    // check if device is on bus and responding
    if(I2C1_PollDevice(TLC59116_Address) < 0){
        return -1;
    }
    
    // send soft reset command, this resets all config and loads calibration
    I2C1_Write(  TLC59116_Address,\
                    0xFF,\
                    (uint8_t *)&TLC59116_Mode2_Clear[1],\
                    TLC59116_Mode2_Clear[0]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    0xFF,\
                    (uint8_t *)&TLC59116_Mode2_Clear[1],\
                    TLC59116_Mode2_Set[0]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    0xFF,\
                    (uint8_t *)&TLC59116_Mode1[1],\
                    TLC59116_Mode1[0]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    0xFF,\
                    (uint8_t *)&TLC59116_GRP_Duty_Cycle[1],\
                    TLC59116_GRP_Duty_Cycle[0]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    0xFF,\
                    (uint8_t *)&TLC59116_LEDOut0[1],\
                    TLC59116_LEDOut0[0]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    0xFF,\
                    (uint8_t *)&TLC59116_LEDOut1[1],\
                    TLC59116_LEDOut0[0]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    0xFF,\
                    (uint8_t *)&TLC59116_LEDOut2[1],\
                    TLC59116_LEDOut0[0]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    0xFF,\
                    (uint8_t *)&TLC59116_LEDOut3[1],\
                    TLC59116_LEDOut0[0]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    0xFF,\
                    (uint8_t *)&TLC59116_IRef[1],\
                    TLC59116_IRef[0]\
                    );
    return 0;
}
