/*
 * Blank template for PIC24FJ128GB204, with:
 *      - serial port UART1
 *      - pwm heartbeat LED
 *      - Green and RED LED
 *      - TP1
 */

#include "config.h"
#include "main.h"
#include "TLC59116.h"

bool RotPush_GoneHigh, RotPush_GoneLow = false;  // for the rotary pushbutton
bool Rotated_CCW, Rotated_CW = false;
structStripDef currentStripDef, nextStripDef;
extern volatile bool Flag2ms;  // Flag to run the debouncing routine
extern volatile bool Flag100ms;     // Used in interrupt 2 to generate 500ms flag
uint8_t RotSM,RotSM_OLD = 2;




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
                    uint8_t i;
                    for (i=0; i < 16; i++){
                        nextStripDef.ledBrightness[i] = nextStripDef.ledBrightness[i] + 0x10;
                    }
                    nextStripDef.hasBeenUpdated = 1;
                }
                if(RotSM_OLD == 3){
                    InputChanged = true;
                    Rotated_CW = true;
                    uint8_t i;
                    for (i=0; i < 16; i++){
                        nextStripDef.ledBrightness[i] = nextStripDef.ledBrightness[i] - 0x10;
                    }
                    nextStripDef.hasBeenUpdated = 1;
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
void setLedsManual ( uint8_t value){
    uint8_t i = 1;
    for (i=1;i<17;i++){
        setLed(i,value);
    }
}

void initialFillStruct ( structStripDef * stripDef ){
    uint8_t i;
    stripDef->center = 8;
    stripDef->width = 16;
    stripDef->masterBrigtness = 0x7F;
    for (i = 0; i < 15; i++){
        stripDef->ledBrightness[i] = 0xFF;
    }
    stripDef->hasBeenUpdated = 0;
}

void checkNextStripDef (void){
    if(nextStripDef.hasBeenUpdated){
        setAllLeds(&nextStripDef.ledBrightness[0]);
        nextStripDef.hasBeenUpdated = 0;
    }
}

int main(void) {
    
    /*Initialize*/
    init_hardware();         // Init all the hardware components
    led_start_routine();         // blink leds
    StartWDT();
    
    /*Init completed*/
    initialFillStruct(&currentStripDef);
    initialFillStruct(&nextStripDef);
    setLedsManual(0x20);
    __delay_ms(50);
    setLedsManual(0x50);
    Uart1SendString("init complete\n");
    uint8_t grpPwm = 0xFF;
    
    /*Main loop*/
    while(1){
        ClrWdt();
                
        if (Flag2ms == true){     // 2ms have passed
            Flag2ms = false;
            if (check_inputs()){
                /* First check pushbutton*/
                if (RotPush_GoneHigh){
                    RotPush_GoneHigh = false;
                    LED_Red_SetLow();
                    LED_Green_SetLow();
                }
                if (RotPush_GoneLow) {
                    /* don't do nothing at the moment, might use this for long press */
                    RotPush_GoneLow = false;
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
        
        if (Flag100ms == true){
            Flag100ms = false;
//            Batt_Raw = sampleBatt();
//            if (!Charging){
//                LED_Red_SetHigh();
//                Uart1SendString("Charging!");
//            }else{
//                LED_Red_SetLow();
//            }
            setGroupPWM(grpPwm);
            grpPwm = grpPwm - 0x7;
            if (grpPwm <= 0x7){
                grpPwm = 0xFF;
            }
        }
        
    }
return 1;
}