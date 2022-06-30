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

// global capture variables
uint16_t ch1Capture[NUM_SAMPLES] = {0};
uint16_t ch2Capture[NUM_SAMPLES] = {0};
uint16_t bitStore[NUM_SAMPLES] = {0};
uint16_t sIndex = 0;
uint16_t tIndex = 0;
volatile bool triggered = false;

volatile bool keepSampling = true;
long samplingTime;
volatile bool hold = false;
// waveform calculated statistics
Stats wStats;

HardwareTimer Timer2(TIM2);
HardwareTimer Timer3(TIM3);
HardwareTimer Timer4(TIM4);
HardwareSerial Serial1(PA10, PA9);

const char *cplNames[] = {"GND", "AC", "DC"};

const char *rngNames[] = {"5V", "2V", "1V", "0.5V", "0.2V", "0.1V", "50mV", "20mV", "10mV"};
const float adcMultiplier[] = {0.05085, 0.02034, 0.01017, 0.005085, 0.002034, 0.001017, 0.5085, 0.2034, 0.1017};
// analog switch enumerated values
uint8_t couplingPos, rangePos;

// this represents the offset voltage at ADC input (1.66V), when Analog input is zero
int16_t zeroVoltageA1, zeroVoltageA2;

// timebase enumerations and store
const char *tbNames[] = {"20 us", "30 us", "50 us", "0.1 ms", "0.2 ms", "0.5 ms", "1 ms", "2 ms", "5 ms", "10 ms", "20 ms", "50 ms"};
uint8_t currentTimeBase;


// ------------------------
void setup()	{
// ------------------------
    __IO uint32_t* mapr = (uint32_t*)0x40010004;//&AFIO_BASE->MAPR;
    *mapr = (*mapr & ~AFIO_MAPR_SWJ_CFG) | 2;

    //afio_cfg_debug_ports(/*AFIO_DEBUG_NONE*/ AFIO_DEBUG_SW_ONLY); //added to disable the debug port. My stock DSO-138 won't allow the screen to work without this
	// see http://www.stm32duino.com/viewtopic.php?t=1130#p13919 for more info
    HAL_Init();
	
	DBG_INIT(SERIAL_BAUD_RATE);
	DBG_PRINT("Dual channel O Scope with two logic channels, ver: ");
	DBG_PRINTLN(FIRMWARE_VERSION);

	// set digital and analog stuff
    initIO();

	// load scope config or factory reset to defaults
    loadConfig(digitalRead(BTN4) == LOW);

	// init the IL9341 display
	initDisplay();
}

void loop(){

	// start by reading the state of analog system
	//readInpSwitches();

	if (triggerType == TRIGGER_AUTO){
		captureDisplayCycle(true);
	}
	else if (triggerType == TRIGGER_NORM){
		captureDisplayCycle(false);
	}
	else{
		// single trigger
		clearWaves();
		indicateCapturing();
		// blocking call - until trigger
		sampleWaves(false);
		indicateCapturingDone();
		hold = true;
		// request repainting of screen labels in next draw cycle
		repaintLabels();
		// draw the waveform
		drawWaves();
		blinkLED();
		// dump captured data on serial port
		//dumpSamples();

		// freeze display
		while (hold);

		// update display indicating hold released
		drawLabels();
	}

	// process any long pending operations which cannot be serviced in ISR
}
