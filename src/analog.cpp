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
    digitalWrite(BOARD_LED, LOW);
}

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    //DBG_PRINTLN("ADC Buffer full");
    digitalWrite(BOARD_LED, HIGH);
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
