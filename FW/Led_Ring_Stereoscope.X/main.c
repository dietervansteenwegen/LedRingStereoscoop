/*
 * Blank template for PIC24FJ128GB204, with:
 *      - serial port UART1
 *      - pwm heartbeat LED
 *      - Green and RED LED
 *      - TP1
 */
// CONFIG1
#pragma config WDTPS = PS512    // Watchdog Timer Postscaler Select->1:512
#pragma config FWPSA = PR128    // WDT Prescaler Ratio Select->1:128
#pragma config WINDIS = OFF    // Windowed WDT Disable->Standard Watchdog Timer
#pragma config FWDTEN = SWON    // Watchdog Timer controlled by SWDTEN
#pragma config ICS = PGx1    // Emulator Pin Placement Select bits->Emulator functions are shared with PGEC1/PGED1
#pragma config LPCFG = OFF    // Low power regulator control->Disabled - regardless of RETEN
#pragma config GWRP = OFF    // General Segment Write Protect->Write to program memory allowed
#pragma config GCP = OFF    // General Segment Code Protect->Code protection is disabled
#pragma config JTAGEN = OFF    // JTAG Port Enable->Disabled

// CONFIG2
#pragma config POSCMD = NONE    // Primary Oscillator Select->Primary Oscillator Disabled
#pragma config WDTCLK = LPRC    // WDT Clock Source Select bits->WDT uses LPRC
#pragma config OSCIOFCN = ON    // OSCO Pin Configuration->OSCO/CLKO/RA3 functions as port I/O (RA3)
#pragma config FCKSM = CSECMD    // Clock Switching and Fail-Safe Clock Monitor Configuration bits->Clock switching is enabled, Fail-Safe Clock Monitor is disabled
#pragma config FNOSC = FRCPLL    // Initial Oscillator Select->Fast RC Oscillator with PLL module (FRCPLL)
#pragma config ALTRB6 = APPEND    // Alternate RB6 pin function enable bit->Append the RP6/ASCL1/PMPD6 functions of RB6 to RA1 pin functions
#pragma config ALTCMPI = CxINC_RB    // Alternate Comparator Input bit->C1INC is on RB13, C2INC is on RB9 and C3INC is on RA0
#pragma config WDTCMX = WDTCLK    // WDT Clock Source Select bits->WDT clock source is determined by the WDTCLK Configuration bits
#pragma config IESO = ON    // Internal External Switchover->Enabled

// CONFIG3
#pragma config WPFP = WPFP127    // Write Protection Flash Page Segment Boundary->Page 127 (0x1FC00)
#pragma config SOSCSEL = ON    // SOSC Selection bits->SOSC circuit selected
#pragma config WDTWIN = PS25_0    // Window Mode Watchdog Timer Window Width Select->Watch Dog Timer Window Width is 25 percent
#pragma config PLLSS = PLL_FRC    // PLL Secondary Selection Configuration bit->PLL is fed by the on-chip Fast RC (FRC) oscillator
#pragma config BOREN = ON    // Brown-out Reset Enable->Brown-out Reset Enable
#pragma config WPDIS = WPDIS    // Segment Write Protection Disable->Disabled
#pragma config WPCFG = WPCFGDIS    // Write Protect Configuration Page Select->Disabled
#pragma config WPEND = WPENDMEM    // Segment Write Protection End Page Select->Write Protect from WPFP to the last page of memory

// CONFIG4
#pragma config DSWDTPS = DSWDTPS1F    // Deep Sleep Watchdog Timer Postscale Select bits->1:68719476736 (25.7 Days)
#pragma config DSWDTOSC = LPRC    // DSWDT Reference Clock Select->DSWDT uses LPRC as reference clock
#pragma config DSBOREN = ON    // Deep Sleep BOR Enable bit->DSBOR Enabled
#pragma config DSWDTEN = ON    // Deep Sleep Watchdog Timer Enable->DSWDT Enabled
#pragma config DSSWEN = ON    // DSEN Bit Enable->Deep Sleep is controlled by the register bit DSEN
#pragma config PLLDIV = PLL6X    // USB 96 MHz PLL Prescaler Select bits->6x PLL selected
#pragma config I2C1SEL = DISABLE    // Alternate I2C1 enable bit->I2C1 uses SCL1 and SDA1 pins
#pragma config IOL1WAY = ON    // PPS IOLOCK Set Only Once Enable bit->Once set, the IOLOCK bit cannot be cleared


#include <xc.h>
#include "main.h"



/* Variables for the heartbeat LED
   PWM cycle is 936 steps, so full off - full on cycle takes
   (936/7)*0.01s = 0.85s */
bool HeartbeatState = 0;              // Heartbeat led state
unsigned int PWMValue = 0x300;  // Start value for PWM duty
signed int PWMStep = 7;         // Step size for PWM
bool CountUp = 1;               // For the PWM of the heartbeat led
unsigned int Counter, Tempcounter =0x0;      // For the RED/GREEN LEDs
unsigned int CountMS = 0x0;
unsigned int Batt_Raw = 0;
bool FlagT2Expired = false;  // Flag to run the debouncing routine
bool Flag500ms = false;     // Used in interrupt 2 to generate 500ms flag
bool RotPush_GoneHigh, RotPush_GoneLow, RotA_GoneHigh, RotA_GoneLow, RotB_GoneHigh, RotB_GoneLow = false;
bool RotCC = false;
bool Rotated_CCW = false;
bool Rotated_CW = false;
unsigned char RotSM = 2;
unsigned char RotSM_OLD = 2;
unsigned char i = 0;            // counter for for loop


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
    FlagT2Expired = true;
    CountMS++;
    if (CountMS == 250){
        Flag500ms = true;
        CountMS = 0;
    }    

}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _ADC1Interrupt( void ){
    IFS0bits.AD1IF=0;
    Uart1SendString("Interrupt\r");
    
}

void start_routine ( void ){
        /* Blink ALL led's for 2 second to indicate reboot */
    #define    FCY    8000000UL    // Instruction cycle frequency, Hz - required for __delayXXX() to work
    #include <libpic30.h>           // has __delay_ms() function
    LED_Red_SetHigh();
    LED_Heartbeat_SetHigh();
    LED_Green_SetHigh();
    for (i=0;i<11;i++){
        LED_Red_Toggle();
        LED_Green_Toggle();
        LED_Heartbeat_Toggle();
        __delay_ms(100);  
    }
}

bool check_inputs ( void ){     // checks status of inputs, returns true if anything has changed
    bool InputChanged = false;
    static char bitmask = 0b11111;      // Only care about the 5 least significant digits
    static unsigned char RotA_State, RotB_State, RotPush_State; // Debounce "map" for RotA, RotB and RotPush
    
    /* RotPush */
    RotPush_State = (((RotPush_State<<1) + RotPush) & bitmask);
            /* Shift previous values to the left, 
             *  add current state at the right,
                then mask to leave only last 5 digits
             */ 
    if ( RotPush_State == 0b10000){
        RotPush_GoneLow = true;
        InputChanged = true;
    }
    if ( RotPush_State == 0b01111){
        InputChanged = true;
        RotPush_GoneHigh = true;
    }
    
    /* State machine describing the status of the rotary encoder
     * Absolute polarity doesn't matter, only changes...
     *
     * Valid states of RotSM:
     * ______________________________________________________
     * |       |                                            |
     * |  "1"  |  B has changed first, assume CCW rotation  |
     * |  "2"  |  Both A and B have same polarity           |
     * |  "3"  |  A has changed first, assume CW rotation   |
     * |_______|____________________________________________|
     * 
     */

    RotSM_OLD = RotSM;
    /* RotA */
    RotA_State = (((RotA_State<<1) + RotA) & bitmask);
            /* Shift previous values to the left, 
             *  add current state at the right,
                then mask to leave only last 5 digits
             */ 
    if (( RotA_State == 0b10000)||( RotA_State == 0b01111)){
        RotSM += 1;
    }
    
    /* RotB */
    RotB_State = (((RotB_State<<1) + RotB) & bitmask);
    if (( RotB_State == 0b10000)||( RotB_State == 0b01111)){
        RotSM -= 1;
    }

    /* If the SM has changed, check the status */
    if (RotSM != RotSM_OLD){
        Uart1SendChar(RotSM + 0x30);
        Uart1SendChar(0x0D);
        
        switch (RotSM){
            case 0:     
                // B has toggled two times in a row, so is back at original position without passing an indent
                RotSM = 2;
                break;
            case 1:     
                // B has toggled, A not yet
                break;
            case 2:     
                // Both have toggled, so position has changed.
                // OR RotSM was 0 or 4, in which case nothing should happen
                if(RotSM_OLD == 1){
                    InputChanged = true;
                    Rotated_CCW = true;
                }
                if(RotSM_OLD == 3){
                    InputChanged = true;
                    Rotated_CW = true;
                }
            case 3:
                // A has toggled, B not yet
                break;
            case 4:
                // A toggled two times, so back at orig. position. Reset to state 2
                RotSM = 2;
                break;
            default:
                // Another value, shouldn't happen so reset
                RotSM = 2;
                break;
        }
    }
    
    if (InputChanged){
        return true;
    }else{
        return false;
    }
    
}


int main(void) {
    
    /*Initialize*/
    init_hardware();         // Init all the hardware components
    start_routine();         // blink leds
    StartWDT();


    /*Init completed*/

    /*Main loop*/
    while(1){
        ClrWdt();
                
        if (FlagT2Expired == true){     // 2ms have passed
            FlagT2Expired = false;
            if (check_inputs()){
                
                /* First check pushbutton*/
                if (RotPush_GoneHigh){
                    RotPush_GoneHigh = false;
                    LED_Red_SetLow();
                    LED_Green_SetLow();
                }
                if (RotPush_GoneLow) {
                    RotPush_GoneLow = false;
                    LED_Red_SetLow();
                    LED_Green_SetLow();
                }
                
                /* Then check rotary encoder */
                if (Rotated_CCW){
                    LED_Red_Toggle();
                    Rotated_CCW = false;
                }
                if (Rotated_CW){
                    LED_Green_Toggle();
                    Rotated_CW = false;
                }
            }
            
        }
        
        if (Flag500ms == true){
            Flag500ms = false;
            Uart1SendString("Sampling...\r");
//            Batt_Raw = sampleBatt();
            Uart1SendString("Value:\r");
//            Uart1SendString(Batt_Raw);
//            Uart1SendChar(Batt_Raw);
            Uart1SendChar('\r');
            if (!Charging){
                LED_Red_SetHigh();
                Uart1SendString("Charging!");
            }else{
                LED_Red_SetLow();
                Uart1SendString("Not charging.");
            }
        }
    }
return 1;
}