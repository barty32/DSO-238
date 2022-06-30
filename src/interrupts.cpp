#include "DSO-238.h"

HardwareTimer Timer5;

static void btnInterruptHandler(Buttons pin){
    static uint32_t lastChange = 0;
    static bool prevState = HIGH;
    uint32_t current = millis();
    bool state = !prevState;

    //debounce
    if(current - lastChange < BTN_DEBOUNCE_TIME){
        return;
    }

    if(state && !prevState){
        btnReleased(pin, current);
    }
    else if(!state && prevState){
        btnPressed(pin, current);
    }

    lastChange = current;
    prevState = state;
}

void okBtnPressed(){
    btnInterruptHandler(BTN_OK);
}

void plusBtnPressed(){
    btnInterruptHandler(BTN_PLUS);
}

void minusBtnPressed(){
    btnInterruptHandler(BTN_MINUS);
}

void selBtnPressed(){
    btnInterruptHandler(BTN_SEL);
}



