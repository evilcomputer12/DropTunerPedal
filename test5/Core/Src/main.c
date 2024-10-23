/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <math.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N 3675
#define halfN N/2

#define BufSize N

int Buf[BufSize];

#define Overlap 2048

int WtrP;
float Rd_P;
float Shift;
float CrossFade;
float a0, a1, a2, b1, b2, hp_in_z1, hp_in_z2, hp_out_z1, hp_out_z2;

#define MAX_DOWN_SHIFT 12

uint32_t semitoneShift = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

CORDIC_HandleTypeDef hcordic;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch2;

FMAC_HandleTypeDef hfmac;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
uint32_t adc_buffer[N];
uint32_t dac_buffer[N];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);
static void MX_CORDIC_Init(void);
static void MX_FMAC_Init(void);
/* USER CODE BEGIN PFP */

// Function to control LEDs based on semitone value
void ControlLEDs(uint16_t semitone)
{
    // Turn off all LEDs before setting new ones
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15 | GPIO_PIN_14, GPIO_PIN_RESET); // LED8 and LED7
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET); // LED6-LED1

    switch (semitone)
    {
        case 7:
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // Light up LED1 (semitone 0)
            break;
        case 6:
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Light up LED2 (semitone 1)
            break;
        case 5:
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET); // Light up LED3 (semitone 2)
            break;
        case 4:
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); // Light up LED4 (semitone 3)
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);  // Light up LED5 (semitone 4)
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);  // Light up LED6 (semitone 5)
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Light up LED7 (semitone 6)
            break;
        case 0:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // Light up LED8 (semitone 7)
            break;
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   // Light up LED6 (for semitones 8-12)
            // Light up corresponding LED for semitone
            switch (semitone)
            {
                case 8:
                	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // Light up LED1 (semitone 8)
                    break;
                case 9:
                	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Light up LED2 (semitone 9)
                    break;
                case 10:
                	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET); // Light up LED3 (semitone 10)
                    break;
                case 11:
                	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET); // Light up LED4 (semitone 11)
                    break;
                case 12:
                	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);  // Light up LED5 (semitone 12)
                    break;
            }
            break;
        default:
            // No action needed for invalid semitone values
            break;
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX_ENCODER_VALUE 65535
#define DESIRED_MAX_VALUE 12
#define SCALING_FACTOR ((MAX_ENCODER_VALUE + 1) / (DESIRED_MAX_VALUE + 1)) // Scaling factor for the range


uint32_t counter = 0;
int16_t count = 0;   // Use signed 16-bit int to handle possible negative values
uint16_t position = 0;
int speed = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    counter = __HAL_TIM_GET_COUNTER(htim);  // Get the raw counter value

    count = (int16_t)counter;  // Cast to signed int to handle negative values

    // If count is negative, set it to zero
    if (count < 0)
    {
        count = 0;
    }

    // Scale down count to position, divide by 4 to adjust for sensitivity
    position = count / 4;

    // Clamp the position to be within 0 to 12
    if (position > 12)
    {
        position = 12;  // Ensure position does not exceed 12
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable clocks for GPIO ports B, D, and A
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Configure GPIO pin outputs for LED8 (PB15) and LED7 (PB14)
  GPIO_InitStruct.Pin = GPIO_PIN_15 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // Push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL;           // No pull-up or pull-down resistors
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // Set low frequency for LEDs
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);       // Initialize GPIOB

  // Configure GPIO pin outputs for LED6 (PD8), LED5 (PD9), LED4 (PD10), LED3 (PD11), LED2 (PD12) , LED2 (PD13)
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  WtrP = 0;
  Rd_P = 0.0f;
  Shift = 0.0f;  // Pitch shift amount (adjust as needed)
  CrossFade = 1.0f;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_CORDIC_Init();
  MX_FMAC_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc3, adc_buffer, N);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, dac_buffer, N, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  semitoneShift = position;
	  ControlLEDs(semitoneShift);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC3_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = ADC3_OVERSAMPLING_RATIO_2;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSign = ADC3_OFFSET_SIGN_NEGATIVE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief FMAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMAC_Init(void)
{

  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 6237;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Pitch shifting function (downward only)

float pitchShiftRatios[13] = {
        1.0f,             // 0 semitone shift
        0.943874f,        // 1 semitone shift
        0.890899f,        // 2 semitone shift
        0.840896f,        // 3 semitone shift
        0.793701f,        // 4 semitone shift
        0.749154f,        // 5 semitone shift
        0.707107f,        // 6 semitone shift
        0.667420f,        // 7 semitone shift
        0.629961f,        // 8 semitone shift
        0.594604f,        // 9 semitone shift
        0.561231f,        // 10 semitone shift
        0.529732f,        // 11 semitone shift
        0.500000f         // 12 semitone shift
    };

/* USER CODE BEGIN 4 */
// Pitch shifting function (downward only)


// Cubic interpolation function
float cubicInterpolate(float y0, float y1, float y2, float y3, float mu) {
    float a0, a1, a2, a3, mu2;

    mu2 = mu * mu;
    a0 = y3 - y2 - y0 + y1;
    a1 = y0 - y1 - a0;
    a2 = y2 - y0;
    a3 = y1;

    return (a0 * mu * mu2 + a1 * mu2 + a2 * mu + a3);
}

// Sine-based smooth crossfade function
float sineCrossfade(float x) {
    return 0.5f * (1.0f - cosf(x * M_PI));  // Smoother transition using cosine
}

// IIR filter coefficients (adjust for the desired cutoff frequency)
float a1 = 0.8f;  // Feedback coefficient
float b0 = 0.2f;  // Feedforward coefficient

// Previous output sample for the IIR filter (initially set to 0)
float previousOutput = 0.0f;

uint32_t Do_PitchShift(uint32_t sample) {
    // Write the original sample to the ring buffer
    Buf[WtrP] = sample;

    // Ensure semitoneShift is within the valid range
    if (semitoneShift > MAX_DOWN_SHIFT) {
        semitoneShift = MAX_DOWN_SHIFT;
    }

    // Calculate the pitch shift ratio (downward shift only)
    float pitchShiftRatio = powf(2.0f, -(float)semitoneShift / 12.0f);

    // Update the read pointer based on the pitch shift ratio
    Rd_P += pitchShiftRatio;

    // Clamp the read pointer to prevent overflow
    if (Rd_P >= BufSize) Rd_P -= BufSize;

    // Compute integer read pointers (main phase and 180° phase)
    int RdPtr_Int = (int)Rd_P % BufSize;               // Main read pointer
    int RdPtr_Int2 = (RdPtr_Int + (BufSize / 2)) % BufSize;  // 180° phase pointer

    // Extract the fractional part of the read pointer for interpolation
    float frac = Rd_P - (float)RdPtr_Int;

    // Get the four samples needed for cubic interpolation
    int RdPtr_Int_M1 = (RdPtr_Int - 1 + BufSize) % BufSize;  // Previous sample (wrap around buffer)
    int RdPtr_Int_P1 = (RdPtr_Int + 1) % BufSize;            // Next sample
    int RdPtr_Int_P2 = (RdPtr_Int + 2) % BufSize;            // Next next sample

    // Cubic interpolation for the main phase
    float Rd0 = cubicInterpolate(
        (float)Buf[RdPtr_Int_M1],
        (float)Buf[RdPtr_Int],
        (float)Buf[RdPtr_Int_P1],
        (float)Buf[RdPtr_Int_P2],
        frac
    );

    // Repeat the same for the 180° phase shift (cross-phase)
    int RdPtr_Int2_M1 = (RdPtr_Int2 - 1 + BufSize) % BufSize;
    int RdPtr_Int2_P1 = (RdPtr_Int2 + 1) % BufSize;
    int RdPtr_Int2_P2 = (RdPtr_Int2 + 2) % BufSize;

    float Rd1 = cubicInterpolate(
        (float)Buf[RdPtr_Int2_M1],
        (float)Buf[RdPtr_Int2],
        (float)Buf[RdPtr_Int2_P1],
        (float)Buf[RdPtr_Int2_P2],
        frac
    );

    // Crossfade logic using overlap and sine-based crossfade
    if (Overlap >= (WtrP - RdPtr_Int) && (WtrP - RdPtr_Int) >= 0 && Shift != 1.0f) {
        int rel = WtrP - RdPtr_Int;
        CrossFade = sineCrossfade((float)rel / (float)Overlap);
    } else if (WtrP - RdPtr_Int == 0) {
        CrossFade = 0.0f;
    }

    if (Overlap >= (WtrP - RdPtr_Int2) && (WtrP - RdPtr_Int2) >= 0 && Shift != 1.0f) {
        int rel = WtrP - RdPtr_Int2;
        CrossFade = 1.0f - sineCrossfade((float)rel / (float)Overlap);
    } else if (WtrP - RdPtr_Int2 == 0) {
        CrossFade = 1.0f;
    }

    // Perform cross-fading and combine the two phase-shifted samples
    float combinedSample = Rd0 * CrossFade + Rd1 * (1.0f - CrossFade);

    // Apply IIR filter to smooth the output
    float filteredSample = b0 * combinedSample + a1 * previousOutput;
    previousOutput = filteredSample;  // Store the filtered output for the next iteration

    // Convert the filtered sample back to integer for output
    sample = (uint32_t)filteredSample;

    // Increment the write pointer and handle wrap-around
    WtrP++;
    if (WtrP == BufSize) WtrP = 0;

    return sample;
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	for(int n = 0; n < halfN; n++)
	{
		dac_buffer[n] = Do_PitchShift(adc_buffer[n]);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	for(int n = halfN; n < N; n++)
	{
		dac_buffer[n] =  Do_PitchShift(adc_buffer[n]);
	}
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
