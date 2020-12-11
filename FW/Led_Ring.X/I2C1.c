/*
 * File:   I2C1.c for PIC24FJ128GB204
 * Author: dieterv
 * v0.3
 * Created on February 3, 2020, 3:09 PM
 */


#include <xc.h>
#include "I2C1.h"
#define FCY 6000000UL    // Instruction cycle frequency, Hz - required for __delayXXX() to work
#include <libpic30.h>
#include "uart.h"

//-------------------Variables-------------------
uint8_t I2C1_State = 1;                         // start with dirty bus
const uint16_t CONST_I2C1_Timeout = 750;        // can possibly be lowered

void __attribute__ ( ( interrupt, no_auto_psv ) ) _MI2C1Interrupt ( void ){
    // I2C1 Master events interrupt
    IFS1bits.MI2C1IF = 0;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _I2C1BCLInterrupt ( void ){
    // I2C1 Bus collision interrupt
    IFS5bits.I2C1BCIF = 0;
}

int8_t I2C1_Init (void){
    // Combination of everything to set up bus
    if(I2C1_InitModule() == I2C1_OK && I2C1_ClearErrors() == I2C1_OK){
        I2C1_Bus_ClrDirty;
        return I2C1_OK;
    }
    return I2C1_Err_BusDirty;
}

int8_t I2C1_InitModule (void) 
    /* Set pins, module config and start module */
{
    I2C1CONLbits.I2CEN = 0;     // Disables module
    __delay_us(10);
    
    /* Set pins digital, input, open drain */
    I2C1_SDA_SetDigital();
    I2C1_SDA_SetDigIn();
    I2C1_SDA_SetOpenDrain();
    I2C1_SCL_SetDigIn();
    I2C1_SCL_SetOpenDrain();
    __delay_us(10);
    
    I2C1_ResetBusPins();            // Toggle clock line to reset any hanging devices
    /* I2C baud rate */
    I2C1BRG = 0x1B;             // FRC = 8MHz, FRCDIV = 4, PLL pre = 6, Fscl = 0.1MHz
    
    /* I2C1CONL/H Control register */
    I2C1CONLbits.I2CSIDL = 0;   // 0 = Continues module operation in Idle mode
    I2C1CONLbits.A10M = 0;      // 0 = I2CADD is a 7-bit slave address
    I2C1CONLbits.DISSLW = 1;    // 1 = Slew rate control is disabled for Standard Speed mode (100 kHz, also disabled for 1 MHz mode)
    IEC1bits.MI2C1IE = 1;		// Enable I2C master interrupt
    I2C1STAT = 0x00;            // Clear all status flags
    
    /* MI2C1 - I2C1 Master Events */
    IFS1bits.MI2C1IF = 0;       // clear the master interrupt flag    
    IEC1bits.MI2C1IE = 1;       // enable the master interrupt

    I2C1CONLbits.I2CEN = 1;     // Enables module and claims pins
    __delay_us(10);
    return I2C1_OK;
}

int8_t I2C1_ResetBusPins(void)
    /* Toggles the clock line a couple of times to reset devices waiting for comm */
{
    uint8_t cycleCLK = 10;
    I2C1CONLbits.I2CEN = 0;     // Disables module, frees pins for manual use
    I2C1_SCL_SetDigOut();
    I2C1_SDA_SetDigOut();
    // Try to get both lines high
    I2C1_SDA_SetHigh();
    I2C1_SCL_SetHigh();
    __delay_us(10);    
    
    // Did SCL go high?
    if(I2C1_SCL_State == 0){
        return I2C1_Err_SCL_Low;
    }
    
//    I2C1_SDA_SetDigIn();
    // Try to get SDA high, if required by toggling SCL
    while (cycleCLK > 0){
        if (I2C1_SDA_State == 1){
            break;                              // succeeded in getting SDA high
        }
        I2C1_SCL_SetLow();
        __delay_us(10);
        I2C1_SCL_SetHigh();
        __delay_us(10);
        cycleCLK--;
    }
    
    // Both should be high by now
    if ((I2C1_SCL_State & I2C1_SDA_State) != 1){
        return I2C1_Err_SDA_Low;                // SDA stuck low
    }
    
    // Send a manual start and stop condition
    I2C1_SDA_SetLow();
    __delay_us(10);
    I2C1_SDA_SetHigh();
    __delay_us(10);
   
    return I2C1_OK;
}

int8_t I2C1_ClearErrors(void){
    // Clear bus collision and receive wait flags
    I2C1STATbits.IWCOL = 0;                     // clear write collision detect
    I2C1STATbits.BCL = 0;                       // clear bus collision detect
    I2C1CONLbits.RCEN = 0;                      // cancel receive wait
    return I2C1_OK;
}

int8_t I2C1_RecoverBus(){
    // try to reset the bus by disabling, performing reset and enabling again
    // returns:
    // I2C1_Err_Hardware
    // I2C1_OK

    // first disable peripheral
    I2C1CONLbits.I2CEN = 0;
    
    // clear errors from stat SFR
    I2C1_ClearErrors();
    
    // then try to reset
    if (I2C1_ResetBusPins() != I2C1_OK){
        return I2C1_Err_Hardware;
    }
        
    // enable again
    I2C1CONLbits.I2CEN = 1;
    __delay_us(1);
    
    I2C1_Bus_ClrDirty;
    return I2C1_OK;
}

int8_t I2C1_CheckBus(){
    if(I2C1_Bus_Dirty){
        if(I2C1_RecoverBus() != I2C1_OK){
            return I2C1_Err_Hardware;
        }
    }
    return I2C1_OK;
}

int8_t I2C1_Start(){
    //Initiates start sequence and waits till finished or timeout
    //Returns:
    //I2C_OK
    //I2C_Err_BCL
    //I2C_Err_IWCOL
    //I2C_Err_TimeoutHW
    
    int timeout = CONST_I2C1_Timeout;
    
    I2C1CONLbits.SEN = 1;           // initiate start condition
    __delay_us(1);
    
    // check and handle bus collision
    if (I2C1STATbits.BCL){          
        I2C1CONLbits.SEN = 0;       // cancel start condition
        I2C1STATbits.BCL = 0;       // clear bus collision flag
        return I2C1_Err_BCL;
    }
    
    // check and handle write collision
    if (I2C1STATbits.IWCOL){
        I2C1CONLbits.SEN = 0;       // cancel start condition
        I2C1STATbits.IWCOL = 0;      // clear write collision flag
        return I2C1_Err_IWCOL;
    }
    
    // wait till finished or timeout
    while (I2C1CONLbits.SEN){
        timeout--;
        if (timeout == 0){
            return I2C1_Err_TimeOut;
        }
    }
    
    return I2C1_OK;
}

int8_t I2C1_Stop(){
    //Initiates stop sequence and waits till finished or timeout
    //Returns:
    //I2C_OK
    //I2C_Err_BCL
    //I2C_Err_SCL_low
    
    int timeout = CONST_I2C1_Timeout ;              // used for the timeout
    
    I2C1CONLbits.PEN = 1;           // initiate stop condition
    __delay_us(1);
    
    // check for bus collision
    if(I2C1STATbits.BCL){
        I2C1STATbits.BCL = 0;       // clear collision flag
        return I2C1_Err_BCL;
    }
    
    // wait for HW to clear PEN (or timeout))
    while(I2C1CONLbits.PEN){            // wait till hw clears bit
        timeout--;
        if (timeout == 0){              // SCL is stuck. 
            return I2C1_Err_SCL_Low;    // Need to reset I2C IF to clear flag.
        }
    }
    return I2C1_OK;
}

int8_t I2C1_Restart(){
    // Initiates repeated start and waits till finished or timeout
    // Returns:
    // I2C_OK
    // I2C_Err_BCL
    // I2C_Err_SCL_low.  SDA stuck low cannot be detected here.
    
    int timeout = CONST_I2C1_Timeout;
    
    I2C1CONLbits.RSEN = 1;              // initiate start condition
    
    // wait until finished
    while(I2C1CONLbits.RSEN){            // bit is cleared by HW when finished
        timeout--;
        if (timeout == 0){              // SCL is stuck low, need IF reset
            return I2C1_Err_SCL_Low;
        }
    }
    
    // check for bus collision
    if(I2C1STATbits.BCL){               // SDA is stuck low
        I2C1STATbits.BCL = 0;           // reset
        return I2C1_Err_BCL;
    }
    
    return I2C1_OK;
}

int8_t I2C1_PollDevice (uint8_t DeviceAddress){
    // Checks state of bus and if device at address is alive
    // Returns:
    // I2C1_OK
    // I2C1_Err_BadAddress
    // I2C1_Err_CommunicationFail
    // I2C1_Err_Hardware
    int ReturnValue;
    uint8_t TargetAddress;
    TargetAddress = (DeviceAddress <<1) & ~0x1; // write address with 0 at end
    
    // Make sure bus is ok first
    if (I2C1_CheckBus() != I2C1_OK){
        return I2C1_Err_BusDirty;
    }
    
    if (I2C1_Start() == I2C1_OK){
        ReturnValue = I2C1_WriteSingleByte(TargetAddress);
        if (I2C1_Stop() == I2C1_OK){
            switch(ReturnValue){
                case I2C1_ACK:
                    return I2C1_OK;
                case I2C1_Err_NAK:
                    return I2C1_Err_BadAddress;
                default:
                    return I2C1_Err_CommunicationFail;
            }
        }else{
            I2C1_Bus_SetDirty;
            return I2C1_Err_CommunicationFail;
        }
    }
    return I2C1_Err_BadAddress;
}

int8_t I2C1_WriteSingleByte(uint8_t cData){
    int timeout = CONST_I2C1_Timeout;
    
    // although this should never happen, check if transmit buffer full
    if (I2C1STATbits.TBF){
        return I2C1_Err_TBF;
    }

    // put data in transmit buffer
	I2C1TRN = cData;                        //Send byte
    
    // wait until finished or timeout
    while(I2C1STATbits.TRSTAT){
        timeout--;
        if (timeout==0){
            return I2C1_Err_SCL_Low;        // need to reset the bus
        }
    }
    
    // check for bus collisions
    if(I2C1STATbits.BCL){
        I2C1STATbits.BCL = 0;               // clear BCL flag
        return I2C1_Err_BCL;
    }
    
    // check the response
    if(I2C1STATbits.ACKSTAT){
//        SendString(4,"NAK\n");
        return I2C1_Err_NAK;                // weirdly enough, ACKSTAT is set for NAK
    }else{
//        SendString(4,"ACK\n");
        return I2C1_OK;
    }
}

int8_t I2C1_Write(uint8_t DeviceAddress, uint8_t SubAddress, const uint8_t *Payload, uint8_t ByteCnt){
    //Writes buffered data to target address/sub address.
    //Returns:
    //I2C_OK
    //I2C_Err_BadAddr
    //I2C_Err_BusDirty
    //I2C_Err_CommFail
    int PayloadByte = 0;
    int ReturnValue = 0;
    uint8_t SlaveWriteAddr;
    
    // Make sure bus is ok first
    if (I2C1_CheckBus() != I2C1_OK){
        return I2C1_Err_BusDirty;
    }
    
    // send start
    if(I2C1_Start() != I2C1_OK){
        I2C1_Bus_SetDirty;
        return I2C1_Err_CommunicationFail;
    }
    
    // address slave
    SlaveWriteAddr = (DeviceAddress << 1) & ~0x1;       // shift left, AND with 0xFE to keep bit0 clear
    ReturnValue = I2C1_WriteSingleByte(SlaveWriteAddr);
    switch (ReturnValue){
        case I2C1_OK:
            break;
        case I2C1_Err_NAK:
            I2C1_Stop();
            I2C1_Bus_SetDirty;
            return I2C1_Err_BadAddress;
        default:
            I2C1_Stop();
            I2C1_Bus_SetDirty;
            return I2C1_Err_CommunicationFail;
            
    }
    
    // send sub address if not 0xF
    if(SubAddress != 0xFF){
        if(I2C1_WriteSingleByte(SubAddress)!= I2C1_ACK){
            I2C1_Stop();
            I2C1_Bus_SetDirty;
            return I2C1_Err_CommunicationFail;
        }
    }
    
    // and finally payload
    for(PayloadByte = 0; PayloadByte < ByteCnt; PayloadByte++){
        // send byte one by one
        if(I2C1_WriteSingleByte(Payload[PayloadByte]) != I2C1_ACK){
            I2C1_Stop();
            I2C1_Bus_SetDirty;
            return I2C1_Err_CommunicationFail;            
        }
    }
    
    // and send stop
    if(I2C1_Stop() != I2C1_OK){
        I2C1_Bus_SetDirty;
        return I2C1_Err_CommunicationFail;
    }
    
    return I2C1_OK;
}    

int8_t I2C1_ReadSingleByte(uint8_t ACKRequired){
    //Initiates read of one byte from slave on I2C bus
    //Slave must already be addressed, and in read mode
    //Waits until completed before returning
    //*Caution* Slave can cause a timeout by clock stretching too long
    // AckRequired: 1 for responding with ACK, 0 for responding with NAK
    //Returns:
    //0x0000-0x00FF Read value stored in low byte (returned integer will always be positive)
    //  Error status is indicated by negative return values
    //I2C_Err_Overflow
    //I2C_Err_RcvTimeout if slave is clock stretching, or SCL shorted to ground
    //I2C_Err_SCL_Low.  SDA stuck low cannot be detected here.
    
    int timeout = CONST_I2C1_Timeout;
    
    // choose response to slave:
    if (ACKRequired > 0){
        I2C1CONLbits.ACKDT = 0;         // respond with ACK
    }else{
        I2C1CONLbits.ACKDT = 1;         // respond with NAK
    }
    
    I2C1CONLbits.RCEN = 1;               // initiate Rx
    while(!I2C1STATbits.RBF){           // set by HW when Rx is finished
        timeout--;
        if (timeout == 0){              // something has gone wrong, not waiting anymore
            return I2C1_Err_RcvTimeOut;
        }
    }
    
    timeout = CONST_I2C1_Timeout;
    I2C1CONLbits.ACKEN = 1;             // respond with ACK/NAK as set above
    while(I2C1CONLbits.ACKEN){
        timeout--;
        if (timeout == 0){              // can happen if SCL is stuck low
            return I2C1_Err_SCL_Low;    // need IF reset to clear ACKEN
        }
    }
    
    if(I2C1STATbits.I2COV){             // new byte came in before the last was read
        I2C1STATbits.I2COV = 0;         // clear flag
        return I2C1_Err_Overflow;
    }
    
    return I2C1RCV;                     // return contents of buffer, reading clears RBF
}

int8_t I2C1_Read(uint8_t DeviceAddress, uint8_t SubAddress, uint8_t *ReadBuffer, uint8_t ByteCnt){
    // Reads data from target into buffer
    // Returns:
    // I2C_Ok
    // I2C_Err_BadAddr
    // I2C_Err_BusDirty
    // I2C_Err_CommFail
    uint8_t SlaveReadAddress;
    uint8_t SlaveWriteAddress;
    uint16_t ReturnValue;
    uint16_t PayloadByte;
    
    // Make sure bus is ok first
    if (I2C1_CheckBus() != I2C1_OK){
        return I2C1_Err_BusDirty;
    }
    
    // send start
    if(I2C1_Start() != I2C1_OK){
        I2C1_Bus_SetDirty;
        return I2C1_Err_CommunicationFail;
    }
    
    // calculate write address
    SlaveWriteAddress = (DeviceAddress << 1) & ~0x1;       // shift left, AND with 0xFE to keep bit0 clear
    
    // address slave and handle reply
    ReturnValue = I2C1_WriteSingleByte(SlaveWriteAddress);
    switch(ReturnValue){
        case I2C1_OK:
            break;
        case I2C1_Err_NAK:
            I2C1_Stop();
            return I2C1_Err_BadAddress;
        default:
            I2C1_Stop();
            return I2C1_Err_CommunicationFail;
    }
    
    // address slave sub address
    if(SubAddress != 0xFF){
        if(I2C1_WriteSingleByte(SubAddress) != I2C1_OK){
            I2C1_Stop();
            I2C1_Bus_SetDirty;
            return I2C1_Err_CommunicationFail;        
        }
    }
    // restart bus
    if(I2C1_Restart()!=I2C1_OK){
        I2C1_Stop();
        I2C1_Bus_SetDirty;
        return I2C1_Err_CommunicationFail; 
    }

    // address the slave read address
    SlaveReadAddress = (DeviceAddress << 1) | 0x01;     // device Address + Read bit
    if(I2C1_WriteSingleByte(SlaveReadAddress) != I2C1_OK){
        I2C1_Stop();
        I2C1_Bus_SetDirty;
        return I2C1_Err_CommunicationFail;   
    }
    
    // read as much bytes as defined
    for(PayloadByte = 0; PayloadByte < ByteCnt; PayloadByte++){
        // the last byte needs a NAK as response, others ACK
        if (PayloadByte == (ByteCnt-1)){
            ReturnValue = I2C1_ReadSingleByte(0);       // send NAK
        }else{
            ReturnValue = I2C1_ReadSingleByte(1);       // send ACK
        }
        if (ReturnValue >= 0){
            ReadBuffer[PayloadByte] = ReturnValue;
        }else{
            I2C1_Stop();
            I2C1_Bus_SetDirty;
            return I2C1_Err_CommunicationFail;  
        }
    }
    
    // and finally send stop
    if(I2C1_Stop() != I2C1_OK){
        I2C1_Bus_SetDirty;
        return I2C1_Err_CommunicationFail;
    }
    
    // all went well
    return I2C1_OK;
}