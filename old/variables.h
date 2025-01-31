// global capture variables
uint16_t ch1Capture[NUM_SAMPLES] = {0};
uint16_t ch2Capture[NUM_SAMPLES] = {0};
uint16_t bitStore[NUM_SAMPLES] = {0};
uint16_t sIndex = 0;
uint16_t tIndex = 0;
volatile boolean triggered = false;

volatile boolean keepSampling = true;
long samplingTime;
volatile boolean hold = false;
// waveform calculated statistics
struct Stats {
	boolean pulseValid;
	double avgPW;
	float duty;
	float freq;
	float cycle;
	
	boolean mvPos;
	float Vrmsf;
	float Vavrf;
	float Vmaxf;
	float Vminf;
} wStats;

enum {CPL_GND, CPL_AC, CPL_DC};
const char* cplNames[] = {"GND", "AC", "DC"}; 
enum {RNG_5V, RNG_2V, RNG_1V, RNG_0_5V, RNG_0_2V, RNG_0_1V, RNG_50mV, RNG_20mV, RNG_10mV};
const char* rngNames[] = {"5V", "2V", "1V", "0.5V", "0.2V", "0.1V", "50mV", "20mV", "10mV"}; 
const float adcMultiplier[] = {0.05085, 0.02034, 0.01017, 0.005085, 0.002034, 0.001017, 0.5085, 0.2034, 0.1017}; 
// analog switch enumerated values
uint8_t couplingPos, rangePos;

// this represents the offset voltage at ADC input (1.66V), when Analog input is zero
int16_t zeroVoltageA1, zeroVoltageA2;

// timebase enumerations and store
enum {T20US, T30US, T50US, T0_1MS, T0_2MS, T0_5MS, T1MS, T2MS, T5MS, T10MS, T20MS, T50MS};
const char* tbNames[] = {"20 us", "30 us", "50 us", "0.1 ms", "0.2 ms", "0.5 ms", "1 ms", "2 ms", "5 ms", "10 ms", "20 ms", "50 ms"}; 
uint8_t currentTimeBase;
