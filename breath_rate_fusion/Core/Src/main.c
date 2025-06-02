/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 256
#define SAMPLE_RATE 20 // Hz
#define FILTER_WINDOW 5
#define BREATH_THRESHOLD 1.0f
#define MIN_BREATH_INTERVAL 1.0f // seconds
#define KALMAN_Q 0.01f // Process noise
#define KALMAN_R 0.1f  // Measurement noise
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t adcBuffer[2]; // For dual channel ADC
float thermistorHistory[BUFFER_SIZE];
float strainHistory[BUFFER_SIZE];
uint32_t sampleTimestamps[BUFFER_SIZE];
uint16_t bufferIndex = 0;

// Kalman filter variables
float kalmanGainThermistor = 0;
float kalmanEstimateThermistor = 0;
float kalmanErrorThermistor = 1;
float kalmanGainStrain = 0;
float kalmanEstimateStrain = 0;
float kalmanErrorStrain = 1;
float fusedBreathRate = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
float readTemperature(uint16_t adcValue);
float readStrain(uint16_t adcValue);
float applyMovingAverageFilter(float newValue, float *history);
void detectBreaths(float *signal, uint32_t *timestamps, uint16_t *count, float *intervals, float threshold);
void updateKalmanFilter(float measurement, float *estimate, float *error, float *gain);
void fuseBreathEstimates(float thermistorRate, float strainRate);
void sendDataViaUART(float breathRate);
void blinkLED();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // Start ADC with DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, 2);

  // Initialize filters
  memset(thermistorHistory, 0, sizeof(thermistorHistory));
  memset(strainHistory, 0, sizeof(strainHistory));

  uint32_t lastSampleTime = 0;
  uint16_t breathCountThermistor = 0;
  uint16_t breathCountStrain = 0;
  float breathIntervalsThermistor[10] = {0};
  float breathIntervalsStrain[10] = {0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t currentTime = HAL_GetTick();
	  // Sample at fixed interval
	  if (currentTime - lastSampleTime >= (1000 / SAMPLE_RATE)) {
		lastSampleTime = currentTime;

		// Read and convert ADC values
		float tempValue = readTemperature(adcBuffer[0]);
		float strainValue = readStrain(adcBuffer[1]);

		// Apply filters
		tempValue = applyMovingAverageFilter(tempValue, thermistorHistory);
		strainValue = applyMovingAverageFilter(strainValue, strainHistory);

		// Store in circular buffer
		thermistorHistory[bufferIndex] = tempValue;
		strainHistory[bufferIndex] = strainValue;
		sampleTimestamps[bufferIndex] = currentTime;
		bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

		// Detect breaths
	  detectBreaths(thermistorHistory, sampleTimestamps, &breathCountThermistor, breathIntervalsThermistor, BREATH_THRESHOLD);
	  detectBreaths(strainHistory, sampleTimestamps, &breathCountStrain, breathIntervalsStrain, BREATH_THRESHOLD);
	  // Calculate breath rates
	  float thermistorRate = 0, strainRate = 0;
	  if (breathCountThermistor >= 2) {
		thermistorRate = 60.0f / (breathIntervalsThermistor[0] / 1000.0f);
		updateKalmanFilter(thermistorRate, &kalmanEstimateThermistor, &kalmanErrorThermistor, &kalmanGainThermistor);
	  }

	  if (breathCountStrain >= 2) {
		strainRate = 60.0f / (breathIntervalsStrain[0] / 1000.0f);
		updateKalmanFilter(strainRate, &kalmanEstimateStrain, &kalmanErrorStrain, &kalmanGainStrain);
	  }

	  // Sensor fusion
		fuseBreathEstimates(kalmanEstimateThermistor, kalmanEstimateStrain);

		// Send data
		sendDataViaUART(1.0f*adcBuffer[0]);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float readTemperature(uint16_t adcValue) {
  // Convert ADC value to temperature using thermistor equation
  // This is a simplified example - you'll need to implement your specific thermistor conversion
  float voltage = adcValue * 3.3f / 4095.0f;
  float resistance = 10000.0f * (3.3f / voltage - 1.0f);
  float steinhart = log(resistance / 10000.0f) / 3950.0f + 1.0f / (25.0f + 273.15f);
  float temperature = (1.0f / steinhart) - 273.15f;
  return temperature;
}

float readStrain(uint16_t adcValue) {
  // Convert ADC value to strain measurement
  // This will depend on your specific strain sensor and amplifier
  return adcValue * 3.3f / 4095.0f;
}

float applyMovingAverageFilter(float newValue, float *history) {
  float sum = newValue;
  for (int i = 0; i < FILTER_WINDOW; i++) {
    sum += history[(bufferIndex - i + BUFFER_SIZE) % BUFFER_SIZE];
  }
  return sum / (FILTER_WINDOW + 1);
}

void detectBreaths(float *signal, uint32_t *timestamps, uint16_t *count, float *intervals, float threshold) {
  static uint8_t lastState = 0;
  static uint32_t lastPeakTime = 0;

  // Simple peak detection algorithm
  float current = signal[bufferIndex];
  float previous = signal[(bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE];
  float next = signal[(bufferIndex + 1) % BUFFER_SIZE];
  // Detect peaks (breath out)
  if (current > previous && current > next && current > threshold) {
    if (lastState == 0) {
      lastState = 1;
      if (lastPeakTime > 0) {
        float interval = timestamps[bufferIndex] - lastPeakTime;
        if (interval > MIN_BREATH_INTERVAL * 1000) {
          // Shift previous intervals
          for (int i = 8; i >= 0; i--) {
            intervals[i+1] = intervals[i];
          }
          intervals[0] = interval;
          (*count)++;
          blinkLED(); // Blink LED on breath detection
        }
      }
      lastPeakTime = timestamps[bufferIndex];
    }
  } else if (current < threshold) {
    lastState = 0;
  }
}

void updateKalmanFilter(float measurement, float *estimate, float *error, float *gain) {
  // Prediction
  float errorPrediction = *error + KALMAN_Q;

  // Update
  *gain = errorPrediction / (errorPrediction + KALMAN_R);
  *estimate = *estimate + *gain * (measurement - *estimate);
  *error = (1 - *gain) * errorPrediction;
}

void fuseBreathEstimates(float thermistorRate, float strainRate) {
  // Simple weighted average - can be made more sophisticated
  // Thermistor gets more weight when it detects breaths
  float weightThermistor = (thermistorRate > 0) ? 0.7f : 0.0f;
  float weightStrain = (strainRate > 0) ? 0.3f : 0.0f;

  if (weightThermistor + weightStrain > 0) {
    fusedBreathRate = (thermistorRate * weightThermistor + strainRate * weightStrain) /
                     (weightThermistor + weightStrain);
  } else {
    fusedBreathRate = 0;
  }
}

void blinkLED(){
	// Toggle LED
	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);

	    // Delay for 500ms
	    HAL_Delay(1000);

	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
}

void sendDataViaUART(float breathRate) {
  char buffer[32];
  int length = snprintf(buffer, sizeof(buffer), "%.2f\n", breathRate);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, length, HAL_MAX_DELAY);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
