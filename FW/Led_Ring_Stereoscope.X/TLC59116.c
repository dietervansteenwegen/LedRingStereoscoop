/*
 * File:   Sensirion_SHT31.c
 * Author: dieterv
 * v0.2
 * Created on February 3, 2020, 3:09 PM
 */

#include <xc.h>
#include "TLC59116.h"
#include "stdint.h"
#include "I2C1.h"
#include "hardware.h"

// Constants (address and commands):
const uint8_t TLC59116_Address = 0x6F;
// constants with commands are in format:
// {register, payload length, payload}
const uint8_t TLC59116_Mode2_Clear[3] = {0x01, 1, 0x80};
const uint8_t TLC59116_Mode2_Set[3] = {0x01, 1, 0x00};
const uint8_t TLC59116_Mode1[3] = {0x00, 1, 0x01};
const uint8_t TLC59116_GRP_Duty_Cycle[3] = {0x12, 1, 0xFF};
const uint8_t TLC59116_LEDOut0[3] = {0x14, 1, 0xFF};
const uint8_t TLC59116_LEDOut1[3] = {0x15, 1, 0xFF};
const uint8_t TLC59116_LEDOut2[3] = {0x16, 1, 0xFF};
const uint8_t TLC59116_LEDOut3[3] = {0x17, 1, 0xFF};
const uint8_t TLC59116_IRef[3] = {0x1C, 1, 0xFF};
const uint8_t TLC59116_PWMxBaseAdd = 0x2;

int8_t TLC59116_InitReset(void){
    // check if device is on bus and responding
//    TP1_Toggle();
    if(I2C1_PollDevice(TLC59116_Address) < 0){
        return -1;
    }
    I2C1_Write(  TLC59116_Address,
                    TLC59116_Mode2_Clear[0],\
                    (uint8_t *)&TLC59116_Mode2_Clear[2],\
                    TLC59116_Mode2_Clear[1]
                    );
    I2C1_Write(  TLC59116_Address,\
                    TLC59116_Mode2_Set[0],\
                    (uint8_t *)&TLC59116_Mode2_Set[2],\
                    TLC59116_Mode2_Set[1]
                    );
    I2C1_Write(  TLC59116_Address,\
                    TLC59116_Mode1[0],\
                    (uint8_t *)&TLC59116_Mode1[2],\
                    TLC59116_Mode1[1]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    TLC59116_GRP_Duty_Cycle[0],\
                    (uint8_t *)&TLC59116_GRP_Duty_Cycle[2],\
                    TLC59116_GRP_Duty_Cycle[1]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    TLC59116_LEDOut0[0],\
                    (uint8_t *)&TLC59116_LEDOut0[2],\
                    TLC59116_LEDOut0[1]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    TLC59116_LEDOut1[0],\
                    (uint8_t *)&TLC59116_LEDOut1[2],\
                    TLC59116_LEDOut1[1]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    TLC59116_LEDOut2[0],\
                    (uint8_t *)&TLC59116_LEDOut2[2],\
                    TLC59116_LEDOut2[1]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    TLC59116_LEDOut3[0],\
                    (uint8_t *)&TLC59116_LEDOut3[2],\
                    TLC59116_LEDOut3[1]\
                    );
    I2C1_Write(  TLC59116_Address,\
                    TLC59116_IRef[0],\
                    (uint8_t *)&TLC59116_IRef[2],\
                    TLC59116_IRef[1]\
                    );
    return 0;
}

uint8_t setLed(uint8_t led, uint8_t brightness){
    if ((led < 1) || (led > 16)){
        return 0;
    }
    if ((brightness < 0) || (brightness > 0xFF)){
        return 0;
    }
    
    I2C1_Write(  TLC59116_Address,\
                    TLC59116_PWMxBaseAdd + (led - 1), \
                    &brightness, \
                    1 \
                    );
    return 1;
}

void setAllLeds(uint8_t * pbrightness1){
    // sets led 1-16 brightness from array starting at * brightness
    // uses the auto-increment flag (first 3 bits 101) to send all brightness values sequentially without 
    // the need for individual adressing.
    // expects 16 values starting at *brightness1
    I2C1_Write(TLC59116_Address, TLC59116_PWMxBaseAdd | 0b10100000, pbrightness1, 16);
}

void setGroupPWM (uint8_t groupPwm){
    I2C1_Write(TLC59116_Address, TLC59116_GRP_Duty_Cycle[0], &groupPwm, 1);
}

