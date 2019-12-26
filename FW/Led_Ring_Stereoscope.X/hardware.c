/*
 * File:   hardware.c
 * Author: michielt
 *
 * Created on February 15, 2018, 7:27 PM
 */

#include <xc.h>
#include "hardware.h"

void initHardware( void ){
    
    initPins();
    initPPS();
    initOscillator();
    initInterrupts();
    InitU1();
    initTimer1();
    initTimer2();
}

void initPins( void ){
    RotPush_SetDigIn();
    RotA_SetDig();
    RotA_SetDigIn();
    RotB_SetDigIn();

    /* TP */    
    TP1_SetDigOut();
    TP1_SetLow();
        
    /* LEDS */
    LED_Red_SetDigOut();
    LED_Green_SetDigOut();
    LED_Heartbeat_SetDigOut();
}

void initPPS( void ){
    __builtin_write_OSCCONL(OSCCON & 0xbf); // Special command to unlock PPS
    
    
    // Outputs are set by assigning a function number to the pin address
    // | RP | address          | function no
    // | 23 | RPOR5bits.RP10R | 3  (U1Tx)
    
    // Inputs are set by setting the pin number to the input function address
    // | Func | address           | RP
    // | U1Rx | RPINR18bits.U1RXR | 11 (0x0B)
    
    //UART1    
    RPOR5bits.RP10R = 0x0003;    // (U1Tx) RP23 pin gets function 3 (U1Tx)
    RPINR18bits.U1RXR = 0x000B;    //(U1Rx) Function U1Rx to pin 0x26 (rp22)
    
    __builtin_write_OSCCONL(OSCCON | 0x40); // Special command to lock PPS
}
void initOscillator( void ){
    // CF no clock failure; NOSC FRCPLL; SOSCEN enabled; POSCEN disabled; CLKLOCK unlocked; OSWEN Switch is Complete; IOLOCK not-active; 
    __builtin_write_OSCCONL((uint8_t) (0x0102 & 0x00FF));
    // CPDIV 1:1; PLLEN enabled; RCDIV FRC/4; DOZE 1:8; DOZEN disabled; ROI disabled; 
    CLKDIV = 0x3220;
    // STOR disabled; STORPOL Interrupt when STOR is 1; STSIDL disabled; STLPOL Interrupt when STLOCK is 1; STLOCK disabled; STSRC SOSC; STEN enabled; TUN Center frequency; 
    OSCTUN = 0x8000;
    // ROEN disabled; ROSEL FOSC; ROSIDL disabled; ROSWEN disabled; ROOUT disabled; ROSLP disabled; 
    REFOCONL = 0x0000;
    // RODIV 0; 
    REFOCONH = 0x0000;
    // ROTRIM 0; 
    //REFOTRIML = 0x0000;
}
void initInterrupts (void) {
    /* Enable interrupts */
    IEC1bits.CNIE = 1;    // Interrupt on change enabled
    /* Set priority for interrupts: Rx 1, Tx and Timers 2 */
    IPC3bits.U1TXIP = 2;  //    UTXI: U1TX - UART1 Transmitter    Priority: 2
    IPC2bits.U1RXIP = 1;  //    URXI: U1RX - UART1 Receiver     Priority: 1
    IPC0bits.T1IP = 1;    //    TI: T1 - Timer1    Priority: 2
}

void InitU1(void){
    
    /* UART LINK */
    U1_Tx_SetDigOut();
    U1_Tx_SetHighOut();
    U1_Rx_SetDigIn();   
    
   // STSEL 1; IREN disabled; PDSEL 8N; UARTEN disabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; URXINV disabled; UEN TX_RX; 
   U1MODE = 0x0008;
   // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
   U1STA = 0x0000;
    // BaudRate = 9600; Frequency = 6000000 Hz; U1BRG 77; 
   U1BRG = 0x9B;
   // ADMADDR 0; ADMMASK 0; 
   U1ADMD = 0x0000;
   // T0PD 1 ETU; PTRCL T0; TXRPT Retransmits the error byte once; CONV Direct; SCEN disabled; 
   U1SCCON = 0x0000;
   // TXRPTIF disabled; TXRPTIE disabled; WTCIF disabled; WTCIE disabled; PARIE disabled; GTCIF disabled; GTCIE disabled; RXRPTIE disabled; RXRPTIF disabled; 
   U1SCINT = 0x0000;
   // GTC 0; 
   U1GTC = 0x0000;
   // WTCL 0; 
   U1WTCL = 0x0000;
   // WTCH 0; 
   U1WTCH = 0x0000;

   IEC0bits.U1RXIE = 1;

    //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
   U1MODEbits.UARTEN = 1;  // enabling UART ON bit
   U1STAbits.UTXEN = 1;
   if (RCONbits.WDTO){
       Uart1SendString("---Reset by WDT---\r");
   }
   Uart1SendString("---Init UART1 completed---\r");
}

void initTimer1( void ){
         
    //TMR1 0; 
    TMR1 = 0x0000;
    //TCKPS 1:64; TON disabled; TSIDL disabled; TCS FOSC/2; TECS SOSC; TSYNC disabled; TGATE disabled; 
    T1CON = 0x020;
    // Period: 10ms
    PR1 = 0x3A8;
    
    IEC0bits.T1IE = true;    // Enable the interrupt
    T1CONbits.TON = 1;       // Start Timer
}

void initTimer2( void ){
    // Reset timer2 to 0 
    TMR2 = 0x000;
    //Period around 2ms
    PR2 = 0xBB;
    //T2CON
    T2CON = 0x20;
    
    IEC0bits.T2IE = true;   // Enable the interrupt
    T2CONbits.TON = 1;      // Start the timer
}