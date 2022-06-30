#pragma once

// comment out following line to use DSO push buttons instead of encoder
//#define USE_ENCODER

//---------------------------------------------------------------------------------------------------------------------------------
//
//                                                    Includes
//
//---------------------------------------------------------------------------------------------------------------------------------

#include <Arduino.h>
#include <EEPROM.h>

//#include <Adafruit_GFX.h> -> included in <Adafruit_TFTLCD_8bit_STM32.h>
// needs to be Adafruit GFX Library v1.1.4, check/change your installed version
// otherwise you will get a black screen or compiler errors

#include <Adafruit_TFTLCD_8bit_STM32.h>

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

// display colours
#define AN_SIGNAL1 		ILI9341_YELLOW
#define AN_SIGNAL2 		ILI9341_MAGENTA
#define DG_SIGNAL1 		ILI9341_RED
#define DG_SIGNAL2 		ILI9341_BLUE

// pin definitions (DSO138)
#define BOARD_LED 		PA15
#define TEST_WAVE_PIN 	PA7     // 1KHz square wave output
#define TRIGGER_IN		PA8
#define TRIGGER_LEVEL	PB8
#define VGEN			PB9		// used to generate negative voltage in DSO138

// captured inputs
#define AN_CH1 			PA0		// analog channel 1
#define AN_CH2 			PA4		// analog channel 2
#define DG_CH1 			PA13	// digital channel 1 - 5V tolerant pin. Pin mask throughout code has to match digital pin
#define DG_CH2 			PA14	// digital channel 2 - 5V tolerant pin. Pin mask throughout code has to match digital pin

// misc analog inputs
#define VSENSSEL1 		PA2
#define VSENSSEL2		PA1
#define CPLSEL			PA3

// switches
#define ENCODER_SW		PB12
#define ENCODER_A		PB13
#define ENCODER_B		PB14
#define BTN4 			PB15

// TFT pins are hard coded in Adafruit_TFTLCD_8bit_STM32.h file
// TFT_RD         PB10
// TFT_WR         PC15
// TFT_RS         PC14
// TFT_CS         PC13
// TFT_RST        PB11

// FLASH memory address defines
#define PARAM_PREAMBLE	0
#define PARAM_TIMEBASE	1
#define PARAM_TRIGTYPE	2
#define PARAM_TRIGDIR	3
#define PARAM_XCURSOR	4
#define PARAM_YCURSOR	5	// 5,6,7,8 - 4 params
#define PARAM_WAVES		9	// 9,10,11,12 - 4 params
#define PARAM_TLEVEL	13
#define PARAM_STATS		14
#define PARAM_ZERO1		15
#define PARAM_ZERO2		16

// number of pixels waveform moves left/right or up/down
#define XCURSOR_STEP	50
#define YCURSOR_STEP	10


#define BTN_DEBOUNCE_TIME	300


// TFT display constants
#define PORTRAIT 		0
#define LANDSCAPE 		1

#define TFT_WIDTH		320
#define TFT_HEIGHT		240
#define GRID_WIDTH		300
#define GRID_HEIGHT		210

#define GRID_COLOR		0x4208
#define ADC_MAX_VAL		4096
#define ADC_2_GRID		800


//---------------------------------------------------------------------------------------------------------------------------------
//
//                                                    Structures
//
//---------------------------------------------------------------------------------------------------------------------------------


typedef struct _Stats {
	bool pulseValid;
	double avgPW;
	float duty;
	float freq;
	float cycle;
	
	bool mvPos;
	float Vrmsf;
	float Vavrf;
	float Vmaxf;
	float Vminf;
} Stats;


//---------------------------------------------------------------------------------------------------------------------------------
//
//                                                    Enums
//
//---------------------------------------------------------------------------------------------------------------------------------

enum {TRIGGER_AUTO, TRIGGER_NORM, TRIGGER_SINGLE};

enum {CPL_GND, CPL_AC, CPL_DC};

enum {RNG_5V, RNG_2V, RNG_1V, RNG_0_5V, RNG_0_2V, RNG_0_1V, RNG_50mV, RNG_20mV, RNG_10mV};

enum {T20US, T30US, T50US, T0_1MS, T0_2MS, T0_5MS, T1MS, T2MS, T5MS, T10MS, T20MS, T50MS};
/*
enum{
	T1US,
	T2US,
	T5US,
	T10US,
	T20US, 
	T50US, 
	T0_1MS, 
	T0_2MS, 
	T0_5MS, 
	T1MS, 
	T2MS, 
	T5MS, 
	T10MS,
	T20MS, 
	T50MS,
	T0_1S,
	T0_2S,
	T0_5S,
	T1S,
	T2S,
	T5S,
	T10S,
	T20S,
	T50S,
	T100S,
	T200S,
	T500S
};
*/
enum {L_timebase, L_triggerType, L_triggerEdge, L_triggerLevel, L_waves, L_window, L_vPos1, L_vPos2, L_vPos3, L_vPos4};


//---------------------------------------------------------------------------------------------------------------------------------
//
//                                                    Global variables
//
//---------------------------------------------------------------------------------------------------------------------------------

//#ifdef __cplusplus
#define GLOBAL extern
//#else
//#define GLOBAL
//#endif


// global capture variables
GLOBAL uint16_t ch1Capture[NUM_SAMPLES];
GLOBAL uint16_t ch2Capture[NUM_SAMPLES];
GLOBAL uint16_t bitStore[NUM_SAMPLES];
GLOBAL uint16_t sIndex;
GLOBAL uint16_t tIndex;
GLOBAL volatile bool triggered;

GLOBAL volatile bool keepSampling;
GLOBAL long samplingTime;
GLOBAL volatile bool hold;
// waveform calculated statistics

GLOBAL Stats wStats;

GLOBAL const char* cplNames[]; 

GLOBAL const char* rngNames[]; 

GLOBAL const float adcMultiplier[]; 

// analog switch enumerated values
GLOBAL uint8_t couplingPos, rangePos;

GLOBAL HardwareTimer Timer2;

// this represents the offset voltage at ADC input (1.66V), when Analog input is zero
GLOBAL int16_t zeroVoltageA1, zeroVoltageA2;

// timebase enumerations and store

GLOBAL const char* tbNames[]; 
GLOBAL uint8_t currentTimeBase;



// sampling delay table in quarter-microseconds
GLOBAL const int16_t samplingDelay[];
GLOBAL const uint16_t timeoutDelayMs[];

GLOBAL int16_t sDly, tDly;
GLOBAL bool minSamplesAcquired;
GLOBAL bool triggerRising;
GLOBAL long prevTime;

// hold pointer references for updating variables in memory
GLOBAL uint16_t *sIndexPtr;
GLOBAL volatile bool *keepSamplingPtr;
GLOBAL volatile bool *triggeredPtr;



GLOBAL uint8_t triggerType;



GLOBAL Adafruit_TFTLCD_8bit_STM32 tft;

// rendered waveform data is stored here for erasing
GLOBAL int16_t ch1Old[];
GLOBAL int16_t ch2Old[];
GLOBAL int8_t bitOld[];

// grid variables
GLOBAL const uint8_t hOffset;
GLOBAL const uint8_t vOffset;
GLOBAL const uint8_t dHeight;

// plot variables -- modified by interface section
// controls which section of waveform is displayed on screen
// 0 < xCursor < (NUM_SAMPLES - GRID_WIDTH)
GLOBAL int16_t xCursor;

// controls the vertical positioning of waveform
GLOBAL int16_t yCursors[4];

// controls which waveforms are displayed
GLOBAL bool waves[4];

// prints waveform statistics on screen
GLOBAL bool printStats;

// repaint the labels on screen in draw loop
GLOBAL bool paintLabels;


GLOBAL uint8_t currentFocus;


GLOBAL bool cDisplayed;


GLOBAL int encoderVal;

GLOBAL long lastABPress;


GLOBAL long lastBtnPress;

GLOBAL int16_t trigLevel;

//---------------------------------------------------------------------------------------------------------------------------------
//
//                                                    Function prototypes
//
//---------------------------------------------------------------------------------------------------------------------------------

//capture.cpp
void setSamplingRate(uint8_t timeBase);
void setTriggerRising(bool rising);
//void sampleWaves(int16_t timeoutDelay, int16_t samplingDelay);
void sampleWaves(bool wTimeout);
void startScanTimeout(int16_t mSec);
void triggerISR(void);
void scanTimeoutISR(void);
void startSampling(int16_t lDelay);
inline void snapMicros();
void dumpSamples();
void printSample(uint16_t k, float timeStamp);

//control.cpp
void setTriggerType(uint8_t tType);
void captureDisplayCycle(bool wTimeOut);

//display.cpp
void focusNextLabel();
void repaintLabels();
void initDisplay();
void drawWaves();
void clearWaves();
void indicateCapturing();
void indicateCapturingDone();
void clearNDrawSignals();
inline void plotLineSegment(int16_t transposedPt1, int16_t transposedPt2,  int index, uint16_t color);
void drawVCursor(int channel, uint16_t color, bool highlight);
void drawGrid();
void drawLabels();
void drawStats();
void calculateStats();
void drawVoltage(float volt, int y, bool mvRange);
void clearStats();
void banner();


//encoder.cpp
int getEncoderSteps();
void readEncoderISR();
void readASwitchISR();
void readBSwitchISR();


//interface.cpp
const char* getTimebaseLabel();
void btn4ISR();
void readESwitchISR();
void resetParam();
void calculateTraceZero(int waveID);
void encoderChanged(int steps);
void incrementTLevel();
void decrementTLevel();
void incrementWaves();
void decrementWaves();
void setTriggerRising();
void setTriggerFalling();
void incrementTT();
void decrementTT();
void incrementTimeBase();
void decrementTimeBase();
void setTimeBase(uint8_t timeBase);
void toggleWave(uint8_t num);
void changeYCursor(uint8_t num, int16_t yPos);
void changeXCursor(int16_t xPos);


//io.cpp
void initIO();
void initADC();
void blinkLED();
void initScanTimeout();
int16_t getTriggerLevel();
void setTriggerLevel(int16_t tLvl);
//void readInpSwitches(uint8_t* lpCouplingPos, uint8_t* lpRangePos);
void readInpSwitches();

//zconfig.cpp
void loadConfig(bool reset);
void loadDefaults();
void formatSaveConfig();
void saveParameter(uint16_t param, uint16_t data);





