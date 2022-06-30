#include "DSO-238.h"



bool btnPressed(uint32_t btn, uint32_t time){
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


bool btnReleased(uint32_t btn, uint32_t time){
    DBG_PRINTLN("Btn released");
    digitalToggle(BOARD_LED);
    return true;
}