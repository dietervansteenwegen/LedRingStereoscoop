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
const uint8_t MASTER_BRIGHT_MAX = 0xC0;
const uint8_t MASTER_BRIGHT_STEP = 0x07;
const uint8_t MASTER_BRIGHT_KNEE = 0x50;  // to have more steps at low values
const uint8_t MASTER_BRIGHT_START = 0x50;



bool RotPush_GoneHigh, RotPush_GoneLow = false;  // for the rotary pushbutton
bool Rotated_CCW, Rotated_CW = false;
//structStripDef currentStripDef, nextStripDef;
structStripDef StripDef;
extern volatile bool Flag2ms;  // Flag to run the debouncing routine
extern volatile bool Flag100ms;     // Used in interrupt 2 to generate 500ms flag
uint8_t RotSM,RotSM_OLD = 2;
uint8_t StateMachineState = 0;  // 0 = master brightness, 1 = width, 2 = center

/**
 * recalculateSegment
 * recalculates values for all individual leds depending on center and width
 */
void recalculateSegment (void){
    int8_t segStart,segStop; // start and stop of the segment, might be out of range 1-16
    uint8_t targetLed; // used to hold target led number
    segStart = center - ((width - 1)/2);
    segStop = center + ((width -1)/2);
    if (width == 16){
        segStop++;
    }
    if (segStart < 1){
        segStart = segStart + 16;
        segStop = segStop + 16;
    }
    uint8_t i; // need to do 16 leds
    targetLed = segStart;
    for (i=1;i<17;i++){
        if (targetLed <= segStop){
            StripDef.ledBrightness[(targetLed % 16)] = 0xFF;
        }else
            StripDef.ledBrightness[(targetLed % 16)] = 0x00;
        targetLed++;
    }
    
}

/**
 * updateSegmentSettings
 * updates the width or center of the segment and calculates individual led value
 * set updated = 1
 * @param direction: 1 to increment (wider) -1 to decrement (narrower)
 * @param widthOrCenter: 1  for width, 2 for center
 */
void updateSegmentSettings (int8_t direction, uint8_t widthOrCenter){
    switch (widthOrCenter){
    case 1: //width
        // width should be odd or maximum, so 1/3/5/7/9/11/13/15/16
        switch (direction){
        case 1: //wider
            if(width > 14){
                width = 16;
            }else{
                width = width + 2;
            }
            break;
        case -1: //narrower
            if (width > 15){
                width = 15;
            }else if(width < 2){
                width = 1;
            }else{
                width = width - 2;
            }
            break;
        }
        break;
    case 2: //center
        switch (direction){
        case 1: //increment
            center++; 
            if (center == 17){
                center = 1;
            }
            break;
        case -1: //decrement
            center--; 
            if (center == 0){
                center = 16;
            }
            break;
        }
        break;
    }

    recalculateSegment();
    updated = 1;
}

/**
 * updateMasterBright
 * Updates master brightness value in the struct
 * Uses MASTER_BRIGHT_MAX, MASTER_BRIGHT_STEP, MASTER_BRIGHT_KNEE
 * sets updated = 1
 * @param direction: 1 to increment (brighter) -1 to decrement
 */
void updateMasterBright (int8_t direction){
    switch(direction){
    case 1:     // increment
        /* make sure we can't go higher than MASTER_BRIGHT_MAX */
        if ( masterBright > (MASTER_BRIGHT_MAX - MASTER_BRIGHT_STEP)){
            masterBright = MASTER_BRIGHT_MAX;
        }else{
            if (masterBright > MASTER_BRIGHT_KNEE){
            masterBright += (MASTER_BRIGHT_STEP * 2);
            }else{
            masterBright += (MASTER_BRIGHT_STEP);
            }
        }
        break;
    case -1:
        if (masterBright < MASTER_BRIGHT_STEP){
            masterBright = 0;
        }else{
            if (masterBright > MASTER_BRIGHT_KNEE){
                masterBright -= (MASTER_BRIGHT_STEP * 2);
            }else{
                masterBright -= MASTER_BRIGHT_STEP;
            }
        }
    }
    updated = 1;
}

/**
 * Rotated
 * Handles rotated rotary button depending on current StateMachineState
 * @param int8_t direction: -1 means down, 1 means up
 */
void rotated (int8_t direction){
    switch(StateMachineState){
    case 0:     // master brightness
        updateMasterBright(direction);
        break;
    case 1:     // segment width
        updateSegmentSettings(direction, 1);
        break;
    case 2:     // segment center
        updateSegmentSettings(direction, 2);
        break;
    }
}

/**
 * StateMachineAdvance
 * Updates the state machine state to the next state.
 * States: 0 = master brightness, 1 = width, 2 = center
 */
void stateMachineAdvance ( void ){
    StateMachineState++;
    if (StateMachineState > 2){
        StateMachineState = 0;
    }
}

/**
 * checks status of rotary encoder inputs and updates variables as necessary
 * @return: true if anything has changed, else false
 */
bool check_inputs ( void ){
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

/**
 * setAllLedsManual
 * Test function, individually sets all leds to "value"
 * @param value: value to set all leds to
 */
void setAllLedsManual ( uint8_t value){
    uint8_t i = 1;
    for (i=1;i<17;i++){
        setLed(i,value);
    }
}

/**
 * initialFillStruct
 * Takes an empty structStripDef and sets default values:
 * center = 8
 * width = 16
 * master brightness = MASTER_BRIGHT_START
 * ledBrightness = 0xFF for all led's
 * @param stripDef: the empty struct to be filled
 */
void initialFillStruct ( structStripDef * stripDef ){
    uint8_t i;
    stripDef->cntr = 8;
    stripDef->wdt = 16;
    stripDef->masterBrightness = MASTER_BRIGHT_START;
    for (i = 0; i < 16; i++){
        stripDef->ledBrightness[i] = 0xFF;
    }
    stripDef->hasBeenUpdated = 1;
}

/**
 * updateStrip
 * Sends masterBright + 16 led vals from StripDef to the driver over I2C
 * clears hasBeenUpdated 
 */
void updateStrip (void){
    setGroupPWM(masterBright);
    setAllLeds(&StripDef.ledBrightness[0]);
    updated = 0;
}

int main(void) {
    
    /*Initialize*/
    init_hardware();         // Init all the hardware components
    led_start_routine();         // blink leds
    StartWDT();
    
    /*Init completed*/
    uint16_t Batt_Raw = 0;
    initialFillStruct(&StripDef);
    updateStrip();
    Uart1SendString("init complete\n");
    
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
                    stateMachineAdvance();
                }
                if (RotPush_GoneLow) {
                    /* don't do nothing at the moment, might use this for long press */
                    RotPush_GoneLow = false;
                }
                
                /* Then check rotary encoder */
                if (Rotated_CCW){
                    LED_Red_Toggle();
                    Rotated_CCW = false;
                    rotated(-1);
                    
                }
                if (Rotated_CW){
                    LED_Green_Toggle();
                    Rotated_CW = false;
                    rotated(1);
                }
            }
        }
        
        if (Flag100ms == true){
            Flag100ms = false;
            Batt_Raw = sampleBatt();
//            if (!Charging){
//                LED_Red_SetHigh();
//            }else{
//                LED_Red_SetLow();
//            }
            if (updated){
                updateStrip();
            }
        }
        
    }
    return 1;
}