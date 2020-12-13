/*
 * File:   I2C1.c for PIC24FJ128GB204
 * Author: dieterv
 * v0.3
 * Created on February 3, 2020, 3:09 PM
 */


#ifndef XC_I2C1
#define	XC_I2C1
#include <xc.h> // include processor files - each processor file is guarded. 
//#define FCY 6000000UL    // Instruction cycle frequency, Hz - required for __delayXXX() to work
//#include <libpic30.h>

//--------------------Variables--------------------

//-------------------Macros-------------------
/* I2C Pin macros */
// SCL is on B8, SDA on B9
#define I2C1_SDA_SetDigital()       _ANSB9 = 0
#define I2C1_SDA_SetDigIn()         _TRISB9=1
#define I2C1_SDA_SetDigOut()        _TRISB9=0
#define I2C1_SDA_SetOpenDrain()     ODCBbits.ODB9 = 0
#define I2C1_SDA_SetHigh()          _LATB9 = 1
#define I2C1_SDA_SetLow()           _LATB9 = 0
#define I2C1_SDA_State              PORTBbits.RB9
#define I2C1_SCL_SetDigOut()        _TRISB8=0
#define I2C1_SCL_SetDigIn()         _TRISB8=1
#define I2C1_SCL_SetOpenDrain()     ODCBbits.ODB8 = 0
#define I2C1_SCL_SetHigh()          _LATB8 = 1
#define I2C1_SCL_SetLow()           _LATB8 = 0
//#define I2C1_ToggleClock()          _LATB8 ^= 1
#define I2C1_SCL_State              PORTBbits.RB8

/* I2C MACROS */
//#define I2C1_SendStart()            I2C1CONLbits.SEN = 1
//#define I2C1_SendStop()             I2C1CONLbits.PEN = 1

/* Return values */
#define I2C1_OK                     1
#define I2C1_ACK                    1
#define I2C1_Err_Hardware           -1      // hardware issue 
#define I2C1_Err_SCL_Low            -2      // SCL low: HW issue or slave holding low
#define I2C1_Err_SDA_Low            -3      // SDA low. Possibly held by slave or HW issue
#define I2C1_Err_BCL                -4      // bus collision, reset required  
#define I2C1_Err_IWCOL              -5      // write collision (bus was still busy)
#define I2C1_Err_NAK                -6      // no response from slave, try resending?
#define I2C1_Err_TBF                -7      // transmit buffer full, should never happen since blocking
#define I2C1_Err_Overflow           -9      // new byte came in before previous one is read
#define I2C1_Err_RcvTimeOut         -10     // timeout while waiting for data from slave     
#define I2C1_Err_BusDirty           -100	// need to reset I2C bus 
#define I2C1_Err_TimeOut            -101    // unknown timeout
#define I2C1_Err_CommunicationFail  -102    // vague communication error
#define I2C1_Err_BadAddress         -103    // device address not valid or no response

/* Bus state values */
#define I2C1_Bus_Dirty              (I2C1_State & 0x1)
#define I2C1_Bus_SetDirty           (I2C1_State = 1)
#define I2C1_Bus_ClrDirty           (I2C1_State = 0)

//--------------------Functions--------------------
int8_t I2C1_Init (void);
int8_t I2C1_InitModule (void);
int8_t I2C1_ResetBusPins (void);
int8_t I2C1_ClearErrors(void);
int8_t I2C1_RecoverBus();
int8_t I2C1_CheckBus();
int8_t I2C1_Start ();
int8_t I2C1_Stop ();
int8_t I2C1_Restart();
int8_t I2C1_PollDevice (uint8_t DeviceAddress);
int8_t I2C1_WriteSingleByte(uint8_t);
int8_t I2C1_Write(uint8_t DeviceAddress, uint8_t SubAddr, const uint8_t *Payload, uint8_t ByteCnt);
int8_t I2C1_ReadSingleByte(uint8_t ACKRequired);
int8_t I2C1_Read(uint8_t DeviceAddress, uint8_t SubAddress, uint8_t *ReadBuffer, uint8_t ByteCnt);
#endif	/* XC_HEADER_TEMPLATE_H */