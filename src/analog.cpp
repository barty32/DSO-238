#include "DSO-238.h"


uint16_t channel1[NUM_SAMPLES];

void startSampling(){


    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)channel1, NUM_SAMPLES) != HAL_OK){
        DBG_PRINTLN("ADC DMA Start failed");
    }
}


#ifdef __cplusplus
extern "C" {
#endif


// Called when first half of buffer is filled
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
    //DBG_PRINTLN("ADC Buffer half");
    //digitalWrite(BOARD_LED, LOW);
}

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    //DBG_PRINTLN("ADC Buffer full");
    //digitalWrite(BOARD_LED, HIGH);
    //readInpSwitches();
    blinkLED();
    delay(1000);


    //start sampling again
    startSampling();
}

// This function handles DMA1 channel1 global interrupt.
void DMA1_Channel1_IRQHandler(void){
    HAL_DMA_IRQHandler(&hdma_adc1);
}

// This function handles Hard fault interrupt.
void HardFault_Handler(void){
    while(0){}
    //DBG_PRINTLN("Hard fault");
}

#ifdef __cplusplus
}
#endif


void readInpSwitches(){
    static uint8_t couplingOld, rangeOld;
    uint16_t cpl, pos1, pos2;

    // ADC1 and ADC2 are free running at max speed
    
    hadc1.Instance->SQR3 = 1;//PinMap_ADC[PA0].pi
    delayMicroseconds(100);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    pos1 = HAL_ADC_GetValue(&hadc1) & ADC_DR_DATA;
    
    hadc1.Instance->SQR3 = 2;
    delayMicroseconds(100);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    pos2 = HAL_ADC_GetValue(&hadc1) & ADC_DR_DATA;

    hadc1.Instance->SQR3 = 3;
    delayMicroseconds(100);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    cpl = HAL_ADC_GetValue(&hadc1) & ADC_DR_DATA;

    DBG_PRINT("Pos1: ");
    DBG_PRINT(pos1);
    DBG_PRINT(" Pos2: ");
    DBG_PRINT(pos2);
    DBG_PRINT(" Cpl: ");
    DBG_PRINTLN(cpl);

    if(cpl < 400)
        cplPos = CPL_GND;
    else if(cpl < 2000)
        cplPos = CPL_AC;
    else
        cplPos = CPL_DC;

    if(pos1 < 400)
        rngPos = RNG_1V;
    else if(pos1 < 2000)
        rngPos = RNG_100mV;
    else
        rngPos = RNG_10mV;

    if(pos2 < 400)
        rngPos = static_cast<VerticalRanges>(static_cast<int>(rngPos) - 2);
    else if(pos2 < 2000)
        rngPos = static_cast<VerticalRanges>(static_cast<int>(rngPos) - 1);
    //else
        //rngPos -= 0;

    // check if switch position changed from previous snap
    if(cplPos != couplingOld){
        couplingOld = cplPos;
        drawAnalogSwitchPos(rngPos, cplPos);
    }

    if(rngPos != rangeOld){
        rangeOld = rngPos;
        drawAnalogSwitchPos(rngPos, cplPos);
    }
    
    hadc1.Instance->SQR3 = 0;
}
