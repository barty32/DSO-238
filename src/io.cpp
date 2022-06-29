
#include "DLO-138.h"

int16_t trigLevel = 0;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;


// ------------------------
void initIO()	{
// ------------------------
    // set pin I/O direction
	pinMode(BOARD_LED, OUTPUT);
	pinMode(AN_CH1, INPUT_ANALOG);
	pinMode(AN_CH2, INPUT_ANALOG);
	//pinMode(DG_CH1, INPUT_PULLDOWN);
	//pinMode(DG_CH2, INPUT_PULLDOWN);
	pinMode(TRIGGER_IN, INPUT_PULLUP);
	
	// calibrate the ADC channels at startup
    //adc_calibrate(ADC1);
    //HAL_ADCEx_Calibration_Start(ADC1_BASE);
    //adc_calibrate(ADC2);
    initADC();

	// start 1KHz square wave
	pinMode(TEST_WAVE_PIN, OUTPUT);
	//int overflow = Timer3.setPeriod(1000);
    //pwmWrite(TEST_WAVE_PIN, overflow / 2);//17850
    analogWrite(TEST_WAVE_PIN, 8192);
    //DBG_PRINT("Test square wave started, frequency: "); DBG_PRINTLN(F_CPU / overflow);

	// input button and encoder
	pinMode(ENCODER_SW, INPUT_PULLUP);
	pinMode(ENCODER_A, INPUT_PULLUP);
	pinMode(ENCODER_B, INPUT_PULLUP);
	pinMode(BTN4, INPUT_PULLUP);
	
	attachInterrupt(ENCODER_SW, readESwitchISR, FALLING);
	attachInterrupt(BTN4, btn4ISR, CHANGE);
	
#ifdef USE_ENCODER
	attachInterrupt(ENCODER_A, readEncoderISR, CHANGE);
	attachInterrupt(ENCODER_B, readEncoderISR, CHANGE);
#else
	attachInterrupt(ENCODER_A, readASwitchISR, FALLING);
	attachInterrupt(ENCODER_B, readBSwitchISR, FALLING);
#endif	

	// init trigger level PWM
	// start 20KHz square wave on trigger out reference and negative v gen
    //Timer4.setPeriod(50);
    analogWriteFrequency(20000);
    analogWriteResolution(16);
    pinMode(TRIGGER_LEVEL, OUTPUT);
	pinMode(VGEN, OUTPUT);
    //pwmWrite(VGEN, 700);
    analogWrite(VGEN, 8192);

	//blinkLED();
	
	// init scan timeout timer
	initScanTimeout();
}



void SystemClock_Config(void){
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)  {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)  {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)  {
        Error_Handler();
    }
}



// ------------------------
void initADC()	{
// ------------------------
    /*
    int pinMapADCin1 = 0;//PIN_MAP[AN_CH1].adc_channel;
	//int pinMapADCin2 = 4;//PIN_MAP[AN_CH2].adc_channel;
	
	// opamp is low impedance, set fastest sampling 
	//adc_set_sample_rate(ADC1, ADC_SMPR_1_5);
    //adc_set_sample_rate(ADC2, ADC_SMPR_1_5);
    uint32_t adc_smpr1_val = 0, adc_smpr2_val = 0;
    for(int i = 0; i < 10; i++){
        if(i < 8){
            //ADC_SMPR1 determines sample time for channels [10,17] 
            adc_smpr1_val |= 0 << (i * 3);
        }
        // ADC_SMPR2 determines sample time for channels [0,9] 
        adc_smpr2_val |= 0 << (i * 3);
    }
    ADC1->SMPR1 = adc_smpr1_val;
    ADC1->SMPR2 = adc_smpr2_val;

    //adc_set_reg_seqlen(ADC1, 1);

    uint32_t tmp = ADC1->SQR1;
    tmp &= ~ADC_SQR1_L;
    tmp |= 0 << 20;
    ADC1->SQR1 = tmp;
    
    ADC1->SQR3 = pinMapADCin1;
	// set ADC1 continuous mode
    ADC1->CR2 |= ADC_CR2_CONT;
    //ADC1->CR1 |= ADC_CR1_EOCIE;
    // set ADC2 in regular simultaneous mode
	//ADC1->CR1 |= 0x60000; 		
	ADC1->CR2 |= ADC_CR2_SWSTART;

	// set ADC2 continuous mode
	//ADC2->CR2 |= ADC_CR2_CONT; 	
    //ADC2->SQR3 = pinMapADCin2;
    

    // adc_attach_interrupt(ADC1, ADC_EOC, [](){
    //     DBG_PRINTLN("Interrupt");
    // });
    */

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
    if(HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        //Error_Handler();
        DBG_PRINTLN("ADC Init failed");
    }
    //Configure Regular Channel
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        //Error_Handler();
        DBG_PRINTLN("ADC Channel Init failed");
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
        Error_Handler();
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
    
    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ch1Capture, NUM_SAMPLES) != HAL_OK){
        DBG_PRINTLN("ADC DMA Start failed");
    }

    DBG_PRINTLN("ADC Setup finished");
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


/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void){
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while(1)  {
    }
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void){
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
    while(1)  {
      /* USER CODE BEGIN W1_HardFault_IRQn 0 */
      /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void){
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
    while(1)  {
      /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
      /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void){
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
    while(1)  {
      /* USER CODE BEGIN W1_BusFault_IRQn 0 */
      /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void){
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
    while(1)  {
      /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
      /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void){
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void){
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void){
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}


/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void){
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_adc1);
    /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

    /* USER CODE END DMA1_Channel1_IRQn 1 */
}


#ifdef __cplusplus
}
#endif




// ------------------------
void blinkLED()	{
// ------------------------
	digitalWrite(BOARD_LED, LOW);
	delay(10);
	digitalWrite(BOARD_LED, HIGH);
}

// ------------------------
void initScanTimeout()	{
// ------------------------
    Timer2.setMode(1, TIMER_OUTPUT_COMPARE, NC);
    //Timer2.setChannel1Mode(TIMER_OUTPUTCOMPARE);
	Timer2.pause();
	Timer2.setCaptureCompare(1, 1);
    //Timer2.attachCompare1Interrupt(scanTimeoutISR);
    Timer2.attachInterrupt(scanTimeoutISR);
}



// ------------------------
int16_t getTriggerLevel()	{
// ------------------------
	return trigLevel;
}



// ------------------------
void setTriggerLevel(int16_t tLvl)	{
// ------------------------
	// 600 = 20% duty
	// 1800 = 50%
	trigLevel = tLvl;
    //pwmWrite(TRIGGER_LEVEL, 1800 + trigLevel);
    analogWrite(TRIGGER_LEVEL, 8192 + trigLevel);
}





// ------------------------
void readInpSwitches()	{
// ------------------------
	static uint8_t couplingOld, rangeOld;

    uint16_t cpl, pos1, pos2;

	// ADC1 and ADC2 are free running at max speed
    analogReadResolution(12);
    // change to switch 1 
    //ADC1->SQR3 = 1;
    //delayMicroseconds(100);
    //pos2 = (uint16_t)(ADC1->DR & ADC_DR_DATA);
    pos2 = analogRead(PA1);

    //ADC1->SQR3 = 2;
    //delayMicroseconds(100);
    //pos1 = (uint16_t)(ADC1->DR & ADC_DR_DATA);
    pos1 = analogRead(PA2);

    //ADC1->SQR3 = 3;
    //delayMicroseconds(100);
    //cpl = (uint16_t)(ADC1->DR & ADC_DR_DATA);
    cpl = analogRead(PA3);

    // ADC1regs->SQR3 = PIN_MAP[VSENSSEL2].adc_channel;
	// delayMicroseconds(100);
	// pos2 = (uint16_t) (ADC1regs->DR & ADC_DR_DATA);
	
	// ADC1regs->SQR3 = PIN_MAP[VSENSSEL1].adc_channel;
	// delayMicroseconds(100);
	// pos1 = (uint16_t) (ADC1regs->DR & ADC_DR_DATA);
	
	// ADC1regs->SQR3 = PIN_MAP[CPLSEL].adc_channel;
	// delayMicroseconds(100);
    // cpl = (uint16_t) (ADC1regs->DR & ADC_DR_DATA);

	if(cpl < 400)
		couplingPos = CPL_GND;
	else if(cpl < 2000)
		couplingPos = CPL_AC;
	else
		couplingPos = CPL_DC;
	
	if(pos1 < 400)
		rangePos = RNG_1V;
	else if(pos1 < 2000)
		rangePos = RNG_0_1V;
	else
		rangePos = RNG_10mV;
	
	if(pos2 < 400)
		rangePos -= 2;
	else if(pos2 < 2000)
		rangePos -= 1;
	else
		rangePos -= 0;
	
	// check if switch position changed from previous snap
	if(couplingPos != couplingOld)	{
		couplingOld = couplingPos;
		repaintLabels();
	}
	
	if(rangePos != rangeOld)	{
		rangeOld = rangePos;
		repaintLabels();
	}
	
	// read the negative voltage generator
	// ***
	
	// switch ADC1 back to capture channel
    //ADC1regs->SQR3 = PIN_MAP[AN_CH1].adc_channel;
    //ADC1->SQR3 = 0;
    delayMicroseconds(100);
}
