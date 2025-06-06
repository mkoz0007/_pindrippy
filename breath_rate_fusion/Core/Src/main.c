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
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// --- Bandpass filter state ---
typedef struct {
    float x1, x2;
    float y1, y2;
} BiquadState;

// Structure to hold biquad coefficients
typedef struct {
    float b0, b1, b2;
    float a1, a2;
} BiquadCoeffs;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUFFER_SIZE 256
#define SAMPLE_RATE 20 // Hz
#define FILTER_WINDOW 5
#define BREATH_THRESHOLD 0.1f
#define MIN_BREATH_INTERVAL 1.0f // seconds
// Covariances for each sensor (tune as needed)
#define KALMAN_Q_FUSED 1.0f  // Process noise
#define KALMAN_R_THERMISTOR 3.0f  // Measurement noise for thermistor
#define KALMAN_R_STRAIN     10.0f  // Measurement noise for strain

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

// Add these static variables for the high-pass filter state
static float prevInputThermistor = 0.0f;
static float prevOutputThermistor = 0.0f;
static float prevInputStrain = 0.0f;
static float prevOutputStrain = 0.0f;

// Kalman filter variables
float kalmanFusedEstimate = 0.0f;
float kalmanFusedError = 1.0f;
float fusedBreathRate = 0;

// Biquad filter coefficients
BiquadCoeffs thermistorCoeffs;
BiquadCoeffs strainCoeffs;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
float readTemperature(uint16_t adcValue);
float readVolts(uint16_t adcValue);
float applyMovingAverageFilter(float newValue, float *history);
float highPassFilter(float input, float *prevInput, float *prevOutput, float cutoffHz, float sampleRate);
void detectBreaths(float *signal, uint32_t *timestamps, uint16_t *count, float *intervals, float threshold, uint8_t *breathDetected, uint8_t detectNegativePeak);
void updateFusedKalmanFilter(
    float meas1, float meas2,
    float R1, float R2,
    float *estimate, float *error);
void sendDataViaUART(
    float filteredThermistor,
    float filteredStrain,
    float fusedBreathRate,
    float thermistorRate,
    float strainRate,
    uint8_t breathDetectedThermistor,
    uint8_t breathDetectedStrain
);
void blinkLED();
float biquadBandpass(float x, BiquadState *state, BiquadCoeffs *coeffs);
void calcBiquadBandpassCoeffs(float fs, float f_low, float f_high, BiquadCoeffs *coeffs);
void initFilters();
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
  uint16_t lastBreathCountThermistor = 0;
  uint16_t lastBreathCountStrain = 0;
  uint8_t breathDetectedThermistor = 0;
  uint8_t breathDetectedStrain = 0;
  float breathIntervalsThermistor[10] = {0};
  float breathIntervalsStrain[10] = {0};

  static BiquadState thermistorBPState = {0};
  static BiquadState strainBPState = {0};

  initFilters();
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
		float tempValue = readVolts(adcBuffer[0]);
		float strainValue = readVolts(adcBuffer[1]);

		// Apply filters
		// tempValue = applyMovingAverageFilter(tempValue, thermistorHistory);
		// strainValue = applyMovingAverageFilter(strainValue, strainHistory);

    // Apply bandpass filter to remove drift and noise
    tempValue = biquadBandpass(tempValue, &thermistorBPState, &thermistorCoeffs);
    strainValue = biquadBandpass(strainValue, &strainBPState, &strainCoeffs);
		// Store in circular buffer
		thermistorHistory[bufferIndex] = tempValue;
		strainHistory[bufferIndex] = strainValue;
		sampleTimestamps[bufferIndex] = currentTime;
		bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

		// Detect breaths
	  detectBreaths(thermistorHistory, sampleTimestamps, &breathCountThermistor, breathIntervalsThermistor, -0.15, &breathDetectedThermistor, 1); // negative peak for temp
	  detectBreaths(strainHistory, sampleTimestamps, &breathCountStrain, breathIntervalsStrain, 0.25, &breathDetectedStrain, 0); // positive peak for strain
	  // Calculate breath rates
	  float thermistorRate = 0, strainRate = 0;
	  if (breathCountThermistor >= 2) {
		thermistorRate = 60.0f / (breathIntervalsThermistor[0] / 1000.0f);
	  }

	  if (breathCountStrain >= 2) {
		strainRate = 60.0f / (breathIntervalsStrain[0] / 1000.0f);
	  }

    // Update kalman filter and send over uart if there is a change in breath counts for thermistor and strain
    if (breathCountThermistor != lastBreathCountThermistor && breathCountStrain != lastBreathCountStrain) {
      // Sensor fusion
      updateFusedKalmanFilter(
      thermistorRate, strainRate,
      KALMAN_R_THERMISTOR, KALMAN_R_STRAIN,
      &kalmanFusedEstimate, &kalmanFusedError
      );
      lastBreathCountThermistor = breathCountThermistor;
      lastBreathCountStrain = breathCountStrain;
      }

    // Send data
    sendDataViaUART(
    tempValue,
    strainValue,
    kalmanFusedEstimate,
    thermistorRate,
    strainRate,
    breathDetectedThermistor,
    breathDetectedStrain
);


    /* USER CODE END WHILE */
    }
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

float readVolts(uint16_t adcValue) {
  // Convert ADC value to voltage
  return adcValue * 3.3f / 4095.0f;
}

float applyMovingAverageFilter(float newValue, float *history) {
  float sum = newValue;
  for (int i = 0; i < FILTER_WINDOW; i++) {
    sum += history[(bufferIndex - i + BUFFER_SIZE) % BUFFER_SIZE];
  }
  return sum / (FILTER_WINDOW + 1);
}

// High-pass filter function (single-pole IIR)
// cutoffHz: e.g. 0.1 for slow drift removal, sampleRate: e.g. 20
float highPassFilter(float input, float *prevInput, float *prevOutput, float cutoffHz, float sampleRate) {
    float RC = 1.0f / (2.0f * 3.1415926f * cutoffHz);
    float dt = 1.0f / sampleRate;
    float alpha = RC / (RC + dt);
    float output = alpha * (*prevOutput + input - *prevInput);
    *prevInput = input;
    *prevOutput = output;
    return output;
}

float biquadBandpass(float x, BiquadState *state, BiquadCoeffs *coeffs) {
    float y = coeffs->b0 * x + coeffs->b1 * state->x1 + coeffs->b2 * state->x2
              - coeffs->a1 * state->y1 - coeffs->a2 * state->y2;
    state->x2 = state->x1;
    state->x1 = x;
    state->y2 = state->y1;
    state->y1 = y;
    return y;
}

// Calculate 2nd-order Butterworth bandpass coefficients
void calcBiquadBandpassCoeffs(float fs, float f_low, float f_high, BiquadCoeffs *coeffs) {
    float omega1 = 2.0f * M_PI * f_low / fs;
    float omega2 = 2.0f * M_PI * f_high / fs;
    float bw = omega2 - omega1;
    float omega0 = (omega2 + omega1) / 2.0f;
    float Q = omega0 / bw;

    float alpha = sinf(omega0) / (2.0f * Q);
    float cos_omega0 = cosf(omega0);

    float b0 = alpha;
    float b1 = 0.0f;
    float b2 = -alpha;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_omega0;
    float a2 = 1.0f - alpha;

    // Normalize coefficients
    coeffs->b0 = b0 / a0;
    coeffs->b1 = b1 / a0;
    coeffs->b2 = b2 / a0;
    coeffs->a1 = a1 / a0;
    coeffs->a2 = a2 / a0;
}

void initFilters() {
    // For 20Hz sample rate, 0.05Hz - 1Hz band
    calcBiquadBandpassCoeffs(SAMPLE_RATE, 0.01f, 1.0f, &thermistorCoeffs);
    calcBiquadBandpassCoeffs(SAMPLE_RATE, 0.01f, 1.0f, &strainCoeffs);
}

void detectBreaths(
    float *signal,
    uint32_t *timestamps,
    uint16_t *count,
    float *intervals,
    float threshold,
    uint8_t *breathDetected,
    uint8_t detectNegativePeak)
{
    // Make lastState and lastPeakTime static per function call (per sensor)
    static uint8_t lastStateThermistor = 0;
    static uint8_t lastStateStrain = 0;
    static uint32_t lastPeakTimeThermistor = 0;
    static uint32_t lastPeakTimeStrain = 0;

    int prev = (bufferIndex - 3 + BUFFER_SIZE) % BUFFER_SIZE;
    int curr = (bufferIndex - 2 + BUFFER_SIZE) % BUFFER_SIZE;
    int next = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;

    float previous = signal[prev];
    float current  = signal[curr];
    float nextVal  = signal[next];

    *breathDetected = 0; // Default to no breath detected

    // Use separate state for each sensor
    uint8_t *lastState = detectNegativePeak ? &lastStateThermistor : &lastStateStrain;
    uint32_t *lastPeakTime = detectNegativePeak ? &lastPeakTimeThermistor : &lastPeakTimeStrain;

    if (detectNegativePeak) {
        // Detect negative peaks (e.g., inhale for thermistor)
        if (current < previous && current < nextVal && current < threshold) {
            if (*lastState == 0) {
                *lastState = 1;
                if (*lastPeakTime > 0) {
                    float interval = timestamps[curr] - *lastPeakTime;
                    if (interval > MIN_BREATH_INTERVAL * 1000) {
                        for (int i = 8; i >= 0; i--) {
                            intervals[i+1] = intervals[i];
                        }
                        intervals[0] = interval;
                        (*count)++;
                        *breathDetected = 1;
                        *lastPeakTime = timestamps[curr];
                    }
                } else {
                    *lastPeakTime = timestamps[curr];
                }
            }
        } else if (current > threshold) {
            *lastState = 0;
        }
    } else {
        // Detect positive peaks (e.g., exhale for strain)
        if (current > previous && current > nextVal && current > threshold) {
            if (*lastState == 0) {
                *lastState = 1;
                if (*lastPeakTime > 0) {
                    float interval = timestamps[curr] - *lastPeakTime;
                    if (interval > MIN_BREATH_INTERVAL * 1000) {
                        for (int i = 8; i >= 0; i--) {
                            intervals[i+1] = intervals[i];
                        }
                        intervals[0] = interval;
                        (*count)++;
                        *breathDetected = 1;
                        *lastPeakTime = timestamps[curr];
                    }
                } else {
                    *lastPeakTime = timestamps[curr];
                }
            }
        } else if (current < threshold) {
            *lastState = 0;
        }
    }
}

// --- Multisensor Kalman update function ---
void updateFusedKalmanFilter(
    float meas1, float meas2,
    float R1, float R2,
    float *estimate, float *error)
{
    // Prediction
    float errorPrediction = *error + KALMAN_Q_FUSED;

    // If both measurements are valid (>0), fuse both
    if (meas1 > 0 && meas2 > 0) {
        // Combined measurement and noise
        float z = (meas1 / R1 + meas2 / R2) / (1.0f / R1 + 1.0f / R2);
        float R = 1.0f / (1.0f / R1 + 1.0f / R2);

        // Kalman gain
        float K = errorPrediction / (errorPrediction + R);

        // Update
        *estimate = *estimate + K * (z - *estimate);
        *error = (1 - K) * errorPrediction;
    }
    // Only thermistor valid
    else if (meas1 > 0) {
        float K = errorPrediction / (errorPrediction + R1);
        *estimate = *estimate + K * (meas1 - *estimate);
        *error = (1 - K) * errorPrediction;
    }
    // Only strain valid
    else if (meas2 > 0) {
        float K = errorPrediction / (errorPrediction + R2);
        *estimate = *estimate + K * (meas2 - *estimate);
        *error = (1 - K) * errorPrediction;
    }
    // No valid measurement, prediction only
    else {
        *error = errorPrediction;
    }
}

void blinkLED(){
	// Toggle LED
	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);

	    // Delay for 500ms
	    HAL_Delay(1000);

	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
}

// Update sendDataViaUART to include the detection flags:
void sendDataViaUART(
    float filteredThermistor,
    float filteredStrain,
    float fusedBreathRate,
    float thermistorRate,
    float strainRate,
    uint8_t breathDetectedThermistor,
    uint8_t breathDetectedStrain
) {
    char buffer[120];
    // Format: thermistor, strain, fused_rate, thermistor_rate, strain_rate, breathDetectedThermistor, breathDetectedStrain\n
    int length = snprintf(buffer, sizeof(buffer), "%.4f,%.4f,%.2f,%.2f,%.2f,%d,%d\n",
                          filteredThermistor, filteredStrain, fusedBreathRate,
                          thermistorRate, strainRate,
                          breathDetectedThermistor, breathDetectedStrain);
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
