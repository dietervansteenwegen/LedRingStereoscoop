#include <stdint.h>
#include <stdbool.h>    // for bool variable type
#include <xc.h>
#include "hardware.h"

/* Variables for the heartbeat LED
   PWM cycle is 936 steps, so full off - full on cycle takes
   (936/7)*0.01s = 0.85s */
bool HeartbeatState = 0;              // Heartbeat led state
unsigned int PWMValue = 0x300;  // Start value for PWM duty
signed int PWMStep = 7;         // Step size for PWM
bool CountUp = 1;               // For the PWM of the heartbeat led
unsigned int Counter, Tempcounter =0x0;      // For the RED/GREEN LEDs
unsigned int CountMS = 0x0;
bool Flag2ms = false;  // Flag to run the debouncing routine
bool Flag100ms = false;     // Used in interrupt 2 to generate 500ms flag

//bool RotA_GoneHigh, RotA_GoneLow, RotB_GoneHigh, RotB_GoneLow = false; // for the rotary encoder


void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void ){ 
    IFS0bits.U1TXIF = false;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt ( void ){ 
    IFS0bits.U1RXIF = false; // clear interrupt bit

}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _CNInterrupt ( void ){ 
    IFS1bits.CNIF = false;
//    LED_Green_Toggle();
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _T1Interrupt (  ){
/* ISR for TIMER 1 which has two functions:
   - Heartbeat LED_Heartbeat (orange) and bootup led (Red)
        0x3A8 is about 10ms (0.06% error)
        Higher PWMValue: Higher brightness
 */
    IFS0bits.T1IF = false;  // clear interrupt flag
    TMR1 = 0x0000;
    if(HeartbeatState){                   // Led is currently on
        LED_Heartbeat_SetLow();          // switch off
        PR1 = 0x3A8 - PWMValue;     // TMR1 set for off time (10ms - on time)
    }else{
        LED_Heartbeat_SetHigh();
        PR1 = PWMValue;
        if(CountUp){
            PWMValue = PWMValue + PWMStep;
        } else{
            PWMValue = PWMValue - PWMStep;
        }
        if (PWMValue > 0x100){
            PWMStep = 15;   // Value of PWMStep changes depending on duty cycle to compensate for non-linearity of led brightness
        }else{
            PWMStep = 2;
        }
        if(PWMValue>0x390){
            CountUp = 0;     // count down again
        }
        if(PWMValue<10){
            CountUp = 1;     // count up
        }
    }
    HeartbeatState = !HeartbeatState;
    /* Boot led shows that PIC has (re)started... 
     red LED_Boot starts counting down from 200, decrementing each half PWM cycle.
     if it is not 0, decrement 1
     if, after decrementing it is zero, switch off boot led
    */

}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _T2Interrupt (  ){
/* ISR for TIMER 2 which has two functions:
 * - Sets flag every 2ms (0xBB) to let main routine check for button status and debounce
 */
    IFS0bits.T2IF = false;  // clear interrupt flag
    TMR2 = 0x0000;
    Flag2ms = true;
    CountMS++;
    if (CountMS == 50){
        Flag100ms = true;
        CountMS = 0;
    }    

}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _ADC1Interrupt( void ){
    IFS0bits.AD1IF=0;
    Uart1SendString("Interrupt from ADC\r");
    
}
