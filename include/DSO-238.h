#pragma once


//---------------------------------------------------------------------------------------------------------------------------------
//
//                                                    Includes
//
//---------------------------------------------------------------------------------------------------------------------------------

// disable warnings for external libraries
#pragma GCC diagnostic push 
#pragma GCC diagnostic ignored "-Wall"

#include <Arduino.h>
#include <EEPROM.h>

//#include <Adafruit_GFX.h> -> included in <Adafruit_TFTLCD_8bit_STM32.h>
// needs to be Adafruit GFX Library v1.1.4, check/change your installed version
// otherwise you will get a black screen or compiler errors

#include <Adafruit_TFTLCD_8bit_STM32.h>

#pragma GCC diagnostic pop


//---------------------------------------------------------------------------------------------------------------------------------
//
//                                                    Macros
//
//---------------------------------------------------------------------------------------------------------------------------------

#define FIRMWARE_VERSION	"2.0"

// serial print macros
#define DBG_INIT(...) 		{ Serial1.begin(__VA_ARGS__); 	}
#define DBG_PRINT(...) 		{ Serial1.print(__VA_ARGS__); 	}
#define DBG_PRINTLN(...) 	{ Serial1.println(__VA_ARGS__); }

#define SERIAL_BAUD_RATE	115200

// analog and digital samples storage depth
#define NUM_SAMPLES 	2048	

#define BTN_DEBOUNCE_TIME 50



//---------------------------------------------------------------------------------------------------------------------------------
//
//                                                    Structures
//
//---------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------------
//
//                                                    Enums
//
//---------------------------------------------------------------------------------------------------------------------------------

// display colours
enum DisplayColors{
    AN_SIGNAL1 = ILI9341_YELLOW,
    AN_SIGNAL2 = ILI9341_MAGENTA,
    DG_SIGNAL1 = ILI9341_RED,
    DG_SIGNAL2 = ILI9341_BLUE
};

// pin definitions (DSO138)
enum MiscPins{
    BOARD_LED = PA15,
    TEST_WAVE_PIN = PA7,     // 1KHz square wave output
    TRIGGER_IN = PA8,
    TRIGGER_LEVEL = PB8,
    VGEN = PB9		// used to generate negative voltage in DSO138
};

enum Channels{
    AN_CH1 = PA0,   // analog channel 1
    AN_CH2 = PA4,   // analog channel 2
    DG_CH1 = PA13,  // digital channel 1 - 5V tolerant pin. Pin mask throughout code has to match digital pin
    DG_CH2 = PA14   // digital channel 2 - 5V tolerant pin. Pin mask throughout code has to match digital pin
};

#define GET_CHANNEL(channel) (enabledWaves & (1 << (channel)))
#define SET_CHANNEL(channel, on) (enabledWaves &= ~(1 << (channel)))

enum ChannelMask{
    ANALOG1,
    ANALOG2,
    DIGITAL1,
    DIGITAL2
};

enum AnalogSwitches{
    VSENSSEL1 = PA2,
    VSENSSEL2 = PA1,
    CPLSEL = PA3
};

enum Buttons{
    BTN_OK = PB15,
    BTN_PLUS = PB14,
    BTN_MINUS = PB13,
    BTN_SEL = PB12
};

enum TriggerType{
    TRIGGER_AUTO,
    TRIGGER_NORM,
    TRIGGER_SINGLE
};

enum CouplingType{
    CPL_GND,
    CPL_AC,
    CPL_DC
};

enum VerticalRanges{
    RNG_5V,
    RNG_2V,
    RNG_1V,
    RNG_500mV,
    RNG_200mV,
    RNG_100mV,
    RNG_50mV,
    RNG_20mV,
    RNG_10mV
};

enum TimeBases{
    T500S,
    T200S,
    T100S,
    T50S,
    T20S,
    T10S,
    T5S,
    T2S,
    T1S,
    T0_5S,
    T0_2S,
    T0_1S,
    T50MS,
    T20MS,
    T10MS,
    T5MS,
    T2MS,
    T1MS,
    T0_5MS,
    T0_2MS,
    T0_1MS,
    T50US,
    T20US,
    T10US,
    T5US,
    T2US,
    T1US
};

enum Pages{
    PG_MAIN,
    PG_SETTINGS
};

enum MainCursorPos{
    L_timebase,
    L_triggerType,
    L_triggerEdge,
    L_settings,
    L_triggerLevel,
    L_waves,
    L_hPos,
    L_vPos1,
    L_vPos2,
    L_vPos3,
    L_vPos4
};


//---------------------------------------------------------------------------------------------------------------------------------
//
//                                                    Global variables
//
//---------------------------------------------------------------------------------------------------------------------------------

#define GLOBAL extern

GLOBAL ADC_HandleTypeDef hadc1;
GLOBAL DMA_HandleTypeDef hdma_adc1;
GLOBAL Adafruit_TFTLCD_8bit_STM32 tft;

GLOBAL const char* cplNames[];
GLOBAL const char* rngNames[];
GLOBAL const char* tbNames[];


//UI state

//enabled waves mask
//(LSB)
//bit 0: analog 1
//bit 1: analog 2
//bit 3: digital 1
//bit 4: digital 2
//bits 5-7: reserved
GLOBAL volatile uint8_t enabledWaves;
GLOBAL volatile CouplingType cplPos;
GLOBAL volatile VerticalRanges rngPos;
GLOBAL volatile TimeBases currentTimeBase;
GLOBAL volatile TriggerType triggerType;
GLOBAL volatile bool triggerRising;
GLOBAL volatile Pages currentPage;
GLOBAL volatile MainCursorPos currentFocus;


//sample buffer
GLOBAL uint16_t channel1[NUM_SAMPLES];





//---------------------------------------------------------------------------------------------------------------------------------
//
//                                                    Function prototypes
//
//---------------------------------------------------------------------------------------------------------------------------------

//hardware.cpp
bool initHardware();
bool initSystemClock();
bool initADC();
void blinkLED();


//interrupts.cpp
void okBtnPressed();
void plusBtnPressed();
void minusBtnPressed();
void selBtnPressed();


//ui.cpp
bool btnPressed(Buttons btn, uint32_t time);
bool btnReleased(Buttons btn, uint32_t time);
void moveFocus();
void changeTimeBase(bool decrease);

//analog.cpp
void startSampling();
void readInpSwitches();

//display.cpp
void initDisplay();
void banner();
void drawGrid();
void clearWaves();
void drawAllLabels();
bool redrawLabel(MainCursorPos which, TimeBases timeBase, TriggerType trigType, bool rising, uint16_t xPos);
void drawAnalogSwitchPos(VerticalRanges rngPos, CouplingType cplPos);

