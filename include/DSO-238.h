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

enum AnalogSwitches{
    VSENSSEL1 = PA2,
    VSENSSEL2 = PA1,
    CPLSEL = PA3
};

enum Buttons{
    BTN_OK = PB12,
    BTN_PLUS = PB13,
    BTN_MINUS = PB14,
    BTN_SEL = PB15
};


//---------------------------------------------------------------------------------------------------------------------------------
//
//                                                    Global variables
//
//---------------------------------------------------------------------------------------------------------------------------------

#define GLOBAL extern

GLOBAL ADC_HandleTypeDef hadc1;
GLOBAL DMA_HandleTypeDef hdma_adc1;


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
bool btnPressed(uint32_t btn, uint32_t time);
bool btnReleased(uint32_t btn, uint32_t time);

//analog.cpp
void startSampling();

