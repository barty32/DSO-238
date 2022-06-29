#include "DLO-138.h"

uint8_t triggerType;


// ------------------------
void setTriggerType(uint8_t tType)	{
// ------------------------
	triggerType = tType;
	// break any running capture loop
	keepSampling = false;
}





// ------------------------
void captureDisplayCycle(bool wTimeOut)	{
// ------------------------
	indicateCapturing();
	// blocking call - until timeout or trigger
	sampleWaves(wTimeOut);
	// draw the waveform
	indicateCapturingDone();
	drawWaves();
	// inter wait before next sampling
	if(triggered)
		blinkLED();
	
	if(hold)	{
		// update UI labels
		drawLabels();
		// dump captured data on serial port
		//dumpSamples();
	}
	
	// freeze display if requested
	while(hold);
}
