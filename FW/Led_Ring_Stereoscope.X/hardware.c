/*
 * File:   hardware.c
 * Author: michielt
 *
 * Created on February 15, 2018, 7:27 PM
 */

#define  FCY 6000000UL    // Instruction cycle frequency, Hz - required for __delayXXX() to work
#include <xc.h>
#include "hardware.h"
#include "i2c1.h"
#include "TLC59116.h"
#include <libpic30.h>          // For delay functions

void init_hardware( void ){
    
    initPins();
    initPPS();
    initOscillator();
    InitU1();
    initInterrupts();
    initTimer1();
    initTimer2();
    initAdc();
    I2C1_InitModule();
    TLC59116_InitReset();
    led_start_routine();
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
    
    /* CHARGING PIN */
    Charging_SetDig();
    Charging_SetDigIn();
    
    

}

void initPPS( void ){
    __builtin_write_OSCCONL(OSCCON & 0xbf); // Special command to unlock PPS
    
    
    // Outputs are set by assigning a function number to the pin address
    // | RP | address          | function no
    // | 10 | RPOR5bits.RP10R | 3  (U1Tx)
    
    // Inputs are set by setting the pin number to the input function address
    // | Func | address           | RP
    // | U1Rx | RPINR18bits.U1RXR | 11 (0x0B)
    
    //UART1    
    RPOR5bits.RP10R = 0x0003;    // (U1Tx) RP10 pin gets function 3 (U1Tx)
    RPINR18bits.U1RXR = 0x000B;    //(U1Rx) Function U1Rx to pin 0xB (RP11)
    
    __builtin_write_OSCCONL(OSCCON | 0x40); // Special command to lock PPS
}

void initAdc (void){
      /* Battery is connected through voltage divider (xxxR to batt, xxxR to GND) to pin RC1/AN11.
        Pin is initialised in initPins function. */
    
    /* ADC input */
    _ANSC1 = 1;
    _TRISC1 = 1;
    
    AD1CON1bits.ADON = 0;
    AD1CON1bits.ADSIDL = 0;     // Continues operation in idle mode
    AD1CON1bits.DMABM = 0;      // Not used since not using DMA
    AD1CON1bits.DMAEN = 0;      // No extended features
    AD1CON1bits.MODE12 = 0;     // 12 bit operation
    AD1CON1bits.FORM = 0;       // Absolute decimal, unsigned, right justified
    AD1CON1bits.SSRC = 0;       // Clear SAMP bit to start conversion
    AD1CON1bits.ASAM = 0;       // Sample when SAMP is set manually
    AD1CON1bits.SAMP = 0;       // Don't sample yet
    AD1CON1bits.DONE = 0;       // no conversion yet
    
    AD1CON2bits.PVCFG = 0;      // Use AVdd as pos volt ref
    AD1CON2bits.NVCFG0 = 0;     // Use AVss as neg voltage ref
    AD1CON2bits.OFFCAL = 0;     // inputs are connected to normal inputs
    AD1CON2bits.BUFREGEN = 0;   // Buffer register is used as FIFO
    AD1CON2bits.CSCNA = 0;      // No scanning
    AD1CON2bits.BUFS = 0;       // Not used since BUFM = 0
    AD1CON2bits.SMPI = 0;       // Interrupt at completion of each sample (no interrupt needed)
    AD1CON2bits.BUFM = 0;       // Start filling buffer at ADC1BUF0
    AD1CON2bits.ALTS = 0;       // Always use Sample A
    
    AD1CON3bits.ADRC = 0;       // Clock derived from system clock
    AD1CON3bits.PUMPEN = 0;     // Charge pump is disabled
    AD1CON3bits.SAMC = 0b11111; // Auto sample is 31 Tad
    AD1CON3bits.ADCS = 0b1;     // Tad = 2xTcy
    
    //AD1CON4 is used to configure DMA
    //AD1CON5 is used for Threshold detect
    AD1CHS = 0xB;                // select AN11/PORTC1 as POS inp, Vss as neg for CHAN A

    AD1CON1bits.ADON = 1;        // turn ADC ON
    
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

void led_start_routine ( void ){
        /* Blink ALL led's to indicate reboot */
    uint8_t i = 0;

    LED_Red_SetHigh();
    LED_Heartbeat_SetHigh();
    LED_Green_SetHigh();
    for (i=0;i<2;i++){
        LED_Red_Toggle();
        LED_Green_Toggle();
        LED_Heartbeat_Toggle();
        __delay_ms(100);  
    }
}

uint16_t sampleBatt( void ){
    Nop();

    uint16_t value = 0;      // for for loop while sampling
    
    AD1CON1bits.SAMP = 1;               // start sampling
    __delay_ms(100);      
    AD1CON1bits.SAMP = 0;               // stop sampling, start conversion
//    Uart1SendString("Converting\r");
//    __delay_ms(200);      
    while (!AD1CON1bits.DONE){};      // wait for conversion to be finished
//    for(wait=0; wait < 180000; wait++){};    // give time to sample
    value = ADC1BUF0;
    __C30_UART=1;
#include <stdio.h>
    printf("Value: %u\n", value);
    return value;
    snprintf("Value: %u\n", value);
}