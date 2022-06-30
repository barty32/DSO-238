#include "DSO-238.h"


ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;


HardwareSerial Serial1(PA10, PA9);

bool initHardware(){
    DBG_PRINTLN("Initializing hardware...");

    // set pin I/O direction
    pinMode(BOARD_LED, OUTPUT);
    pinMode(AN_CH1, INPUT_ANALOG);
    pinMode(AN_CH2, INPUT_ANALOG);
    //pinMode(DG_CH1, INPUT_PULLDOWN);
    //pinMode(DG_CH2, INPUT_PULLDOWN);
    pinMode(TRIGGER_IN, INPUT_PULLUP);

    // input button and encoder
    pinMode(BTN_SEL, INPUT_PULLUP);
    pinMode(BTN_MINUS, INPUT_PULLUP);
    pinMode(BTN_PLUS, INPUT_PULLUP);
    pinMode(BTN_OK, INPUT_PULLUP);
    
    attachInterrupt(BTN_OK, okBtnPressed, CHANGE);
    attachInterrupt(BTN_PLUS, plusBtnPressed, CHANGE);
    attachInterrupt(BTN_MINUS, minusBtnPressed, CHANGE);
    attachInterrupt(BTN_SEL, selBtnPressed, CHANGE);

    initADC();


    pinMode(TEST_WAVE_PIN, OUTPUT);
    analogWrite(TEST_WAVE_PIN, 8192);

    // init trigger level PWM
    // start 20KHz square wave on trigger out reference and negative v gen
    //Timer4.setPeriod(50);
    analogWriteFrequency(20000);
    analogWriteResolution(16);
    pinMode(TRIGGER_LEVEL, OUTPUT);
    pinMode(VGEN, OUTPUT);
    //pwmWrite(VGEN, 700);
    analogWrite(VGEN, 8192);

    blinkLED();
    return true;
}



bool initSystemClock(){
    
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    // Initializes the RCC Oscillators according to the specified parameters 
    // in the RCC_OscInitTypeDef structure.
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
        return false;
    }
    // Initializes the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
        return false;
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
        return false;
    }
    return true;
}




bool initADC(){

    ADC_ChannelConfTypeDef sConfig = { 0 };

    // DMA interrupt init
    // DMA1_Channel1_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // Common config
    hadc1.Instance = ADC1;
    HAL_ADC_DeInit(&hadc1);
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if(HAL_ADC_Init(&hadc1) != HAL_OK)    {
        return false;
    }
    //Configure Regular Channel
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)    {
        return false;
    }

    __HAL_RCC_DMA1_CLK_ENABLE();
    // ADC1 DMA Init
    // ADC1 Init
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    if(HAL_DMA_Init(&hdma_adc1) != HAL_OK){
        return false;
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);


    // Calibrate ADC
    if(HAL_ADCEx_Calibration_Start(&hadc1)){
        return false;
    }

    // if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ch1Capture, NUM_SAMPLES) != HAL_OK){
    //     DBG_PRINTLN("ADC DMA Start failed");
    // }

    return true;
}




void blinkLED(){
    
    digitalWrite(BOARD_LED, LOW);
    delay(10);
    digitalWrite(BOARD_LED, HIGH);
}
