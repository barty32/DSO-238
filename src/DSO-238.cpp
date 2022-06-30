/*
 * v1.0:
 * - initial release
 * v1.1:
 * - added frequency, cycle, PW and duty units
 * - increased number of steps to move x and y axis
 * - changed stats text color to white
 * - added trigger voltage level readout
 * v2.0:
 * - whole firmware rewrite
 * - use DMA for sampling
*/


#include "DSO-238.h"




void setup(){
    //Disable debug ports
    __IO uint32_t* mapr = (uint32_t*)0x40010004;//&AFIO_BASE->MAPR;
    *mapr = (*mapr & ~AFIO_MAPR_SWJ_CFG) | 2;
    //afio_cfg_debug_ports(/*AFIO_DEBUG_NONE*/ AFIO_DEBUG_SW_ONLY); //added to disable the debug port. My stock DSO-138 won't allow the screen to work without this
    // see http://www.stm32duino.com/viewtopic.php?t=1130#p13919 for more info

    initSystemClock();


    DBG_INIT(SERIAL_BAUD_RATE);
    DBG_PRINT("Dual channel O Scope with two logic channels, ver: ");
    DBG_PRINTLN(FIRMWARE_VERSION);

    initHardware();

    //load config

    //init display
    initDisplay();

    startSampling();
}


void loop(){
    delay(100);
    readInpSwitches();
}


