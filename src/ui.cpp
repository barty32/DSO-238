#include "DSO-238.h"

//UI state

//enabled waves mask
//(LSB)
//bit 0: analog 1
//bit 1: analog 2
//bit 3: digital 1
//bit 4: digital 2
volatile uint8_t enabledWaves = 0b1111;
volatile CouplingType cplPos = CPL_GND;
volatile VerticalRanges rngPos = RNG_5V;
volatile TimeBases currentTimeBase = T500S;
volatile TriggerType triggerType = TRIGGER_AUTO;
volatile bool triggerRising = true;
volatile Pages currentPage = PG_MAIN;
volatile MainCursorPos currentFocus = L_timebase;

bool btnPressed(Buttons btn, uint32_t time){
    // if(current - lastChange < 1000){
    //     DBG_PRINTLN("Short press");
    //     digitalToggle(BOARD_LED);
    // }
    // else if(current - lastChange >= 1000){
    //     DBG_PRINTLN("Long press");
    // }
    DBG_PRINTLN("Btn pressed");
    return true;
}


bool btnReleased(Buttons btn, uint32_t time){

    if(currentPage == PG_MAIN){
        switch(btn){
            case BTN_OK:
                break;
            case BTN_PLUS:
                if(currentFocus == L_timebase){
                    changeTimeBase(false);
                }
                break;
            case BTN_MINUS:
                if(currentFocus == L_timebase){
                    changeTimeBase(true);
                }
                break;
            case BTN_SEL:
                moveFocus();
                break;
        }
    }

    return true;
}


void changeTimeBase(bool decrease){
    uint32_t newTimeBase = static_cast<uint32_t>(currentTimeBase) + (decrease ? -1 : 1);
    if(newTimeBase > T1US){
        return;
    }
    currentTimeBase = static_cast<TimeBases>(newTimeBase);
    redrawLabel(currentFocus, currentTimeBase, triggerType, triggerRising, 0);
}

void moveFocus(){
    uint32_t newFocus = static_cast<uint32_t>(currentFocus) + 1;
    if(!GET_CHANNEL(ANALOG2)){
        ++newFocus;
    }
    if(!GET_CHANNEL(DIGITAL1)){
        ++newFocus;
    }
    if(!GET_CHANNEL(DIGITAL2)){
        ++newFocus;
    }
    if(newFocus > L_vPos4){
        newFocus = 0;
    }
    MainCursorPos prevFocus = currentFocus;
    currentFocus = static_cast<MainCursorPos>(newFocus);
    redrawLabel(prevFocus, currentTimeBase, triggerType, triggerRising, 0);
    redrawLabel(currentFocus, currentTimeBase, triggerType, triggerRising, 0);
}