/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * This code is written for BlackPill STM32F401CCU6 which has USB-C connector.
  * It also has DFU out of the box. But it does not work on Win7 Laptop properly.
  * USB DFU Mode once or twice, When I activated CDC USB VCP, after that Win7
  * cannot identify the DFU device. But the VCP is okay.
  *
  * In This code I am trying to achive Audio In and Out through ADC and DAC. For
  * This I am enabling Timer2 Triggered ADC conversion with DMA support.
  * This code is inspired from https://www.youtube.com/watch?v=AloHXBk6Bfk -
  * Set up multiple ADCs on STM32 microcontrollers using DMA video.
  *
  ******************************************************************************
  */
#include "usbd_cdc_if.h"
extern __IO uint32_t uwTick;
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct __SEQUENCE
{
	int stone1;
	int duration1;
	int stone2;
	int duration2;
	int stone3;
	int duration3;
} PULSEQUENCE;

//PULSEQUENCE pulses = {1000, 10, 1330, 15, 1650, 18};

PULSEQUENCE pulses = {460, 250, 830, 355, 1650, 258};

uint32_t mscount = 0;
uint32_t _pulse_count = 0;

uint8_t usb_rx_buffer[64];
volatile uint8_t flag_usbrx = 0;

char strA1[50];
volatile uint16_t ad1_raw[5];
const int adcChannelCount = 1;
volatile int adcConversionComplete = 0;
volatile uint32_t millis = 0;
volatile uint32_t conv_rate = 0;
volatile uint32_t ad1_audio = 0;

uint32_t buf_idx = 0;
uint8_t buf_num = 1;
uint32_t buf1[5000];
uint32_t buf2[5000];

uint8_t flag_send_bufferA = 0;
uint8_t flag_pulse_out = 0;
uint32_t *buf = buf2;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void DMA_ADC_Complete(DMA_HandleTypeDef *_hdma)
{

}

static float ADC_OLD_Value;
static float P_k1_k1;

//static float Q = 0.0001;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
static float Q = 0.0005;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
//static float R = 0.005; //R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better
static float R = 0.2;
static float Kg = 0;
static float P_k_k1 = 0.5;
static float kalman_adc_old=0;
static int kalman_adc_int = 0;

uint8_t  trip1 = 0;

unsigned long kalman_filter(unsigned long ADC_Value)
{
    float x_k1_k1,x_k_k1;
    //static float ADC_OLD_Value;
    float Z_k;


    float kalman_adc;

    Z_k = ADC_Value;
    x_k1_k1 = kalman_adc_old;

    x_k_k1 = x_k1_k1;
    P_k_k1 = P_k1_k1 + Q;

    Kg = P_k_k1/(P_k_k1 + R);

    kalman_adc = x_k_k1 + Kg * (Z_k - kalman_adc_old);
    P_k1_k1 = (1 - Kg)*P_k_k1;
    P_k_k1 = P_k1_k1;

    ADC_OLD_Value = ADC_Value;
    kalman_adc_old = kalman_adc;
    kalman_adc_int = (int)kalman_adc;
    return kalman_adc;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adcConversionComplete = 1;
	conv_rate++;
	ad1_audio = ad1_raw[0]; //32;
	//TIM1->CCR1 = ad1_audio;

	if(buf_num == 1)
	{
		if(buf_idx < 2000)
		{
			buf1[buf_idx] = ad1_audio;
			// <PWM OUTPUT> TIM1->CCR1 = kalman_filter(buf2[buf_idx]);
			kalman_adc_int = kalman_filter(buf2[buf_idx]);
			buf_idx++;
		}
		else
		{
			buf_num = 2;
			buf_idx = 0;
		}
	}
	else if(buf_num == 2)
	{
		if(buf_idx < 2000)
		{
			buf2[buf_idx] = ad1_audio;
			// <PWM OUTPUT> TIM1->CCR1 = kalman_filter(buf1[buf_idx]);
			kalman_adc_int = kalman_filter(buf1[buf_idx]);
			buf_idx++;
		}
		else
		{
			buf_num = 1;
			buf_idx = 0;
		}
	}

	if(kalman_adc_int > 320)
	{
		trip1 = 1;
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

		//flag_pulse_out = 1;
	}

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ad1_raw, adcChannelCount);

}

/**************************
 * Following callback did not worked. It was referenced from drive.google/ARM/docs.drop/PRIMARY/ARM-CMSIS-7.html
 * Documentation. Please update the docs in the proper places. This callback may work by applying proper
 * configuration. Instead we are using HAL_GetTick() which is same as millis(). To enable this callback to work
 * add HAL_SYSTICK_IRQHandler(); to stm32f4xx_it.c -> void SysTick_Handler(void)
 */
#define MAX_PWM_ALL		20
uint32_t _pwm_slope = 0;
void HAL_SYSTICK_Callback(void)
{
	if(flag_pulse_out == 1)
	{
		millis++;

		if(millis > pulses.stone3)
		{
			if(_pulse_count < pulses.duration3)
			{
				_pulse_count++;
				if(_pulse_count < pulses.duration3)
				{
					if(_pwm_slope < MAX_PWM_ALL)
					{
						_pwm_slope += (MAX_PWM_ALL / pulses.duration3);
						TIM1->CCR3 = _pwm_slope;
					}
				}
				else
				{
					TIM1->CCR3 = 0;

					// END THIS PULSE SEQUENCE
					_pwm_slope = 0;
					flag_pulse_out = 0;
					millis = 0;
					_pulse_count = 0;
					trip1 = 1;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
					//// TEST HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
				}
			}
		}
		else if(millis > pulses.stone2)
		{
			if(_pulse_count < pulses.duration2)
			{
				_pulse_count++;
				if(_pulse_count < pulses.duration2)
				{
					if(_pwm_slope < MAX_PWM_ALL)
					{
						_pwm_slope += ((MAX_PWM_ALL / pulses.duration2) + 1);
						TIM1->CCR1 = _pwm_slope;
					}
				}
				else
				{
					TIM1->CCR1 = 0;
					_pwm_slope = 0;
				}
			}

			if(millis == (pulses.stone2 + 100))
			{
				trip1 = 1;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			}

			if(millis == pulses.stone3)
			{
				_pulse_count = 0;
//				trip1 = 1;
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			}
		}
		else if(millis > pulses.stone1)
		{
			if(_pulse_count < pulses.duration1)
			{
				_pulse_count++;
				if(_pulse_count < pulses.duration1)
				{
					if(_pwm_slope < MAX_PWM_ALL)
					{
						_pwm_slope += ((MAX_PWM_ALL / pulses.duration1) + 1);
						// TIM1->CCR1 = _pwm_slope; // CCR1 should be trip signal
						TIM1->CCR3 = _pwm_slope; 	// CCR3 should be close signal
					}
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
					//// TEST HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
				}
				else
				{
					//TIM1->CCR1 = 0; 	// CCR1 should be trip signal
					TIM1->CCR3 = 0; 	// CCR3 should be close signal
					_pwm_slope = 0;
					trip1 = 1;
					//// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
					//// TEST HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

					//// TEST HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
					//// TEST HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
				}
			}
			else
			{
				////TEST HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
				////TEST HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			}

			if(millis == pulses.stone2)
			{
				_pulse_count = 0;
			}
		}
	}

	if(flag_pulse_out == 2)
		{
			millis++;

			if(millis > pulses.stone3)
			{
				if(_pulse_count < pulses.duration3)
				{
					_pulse_count++;
					if(_pulse_count < pulses.duration3)
					{
						if(_pwm_slope < MAX_PWM_ALL)
						{
							_pwm_slope += (MAX_PWM_ALL / pulses.duration3);
							TIM1->CCR3 = _pwm_slope; // Trip
						}
					}
					else
					{
						TIM1->CCR3 = 0; // Trip

						// END THIS PULSE SEQUENCE
						_pwm_slope = 0;
						flag_pulse_out = 0;
						millis = 0;
						_pulse_count = 0;
						trip1 = 1;
						//// TEST HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
					}
				}
			}
			else if(millis > pulses.stone2)
			{
				if(_pulse_count < pulses.duration2)
				{
					_pulse_count++;
					if(_pulse_count < pulses.duration2)
					{
						if(_pwm_slope < MAX_PWM_ALL)
						{
							_pwm_slope += ((MAX_PWM_ALL / pulses.duration2) + 1);
							TIM1->CCR1 = _pwm_slope; // Trip
						}
					}
					else
					{
						TIM1->CCR1 = 0; // Trip Event
						_pwm_slope = 0;
					}
				}

				if(millis == (pulses.stone2 + 100))
				{
					trip1 = 1;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //// TEST was PIN0
				}

				if(millis == pulses.stone3)
				{
					_pulse_count = 0;
	//				trip1 = 1;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
				}
			}
			else if(millis > pulses.stone1)
			{
				if(_pulse_count < pulses.duration1)
				{
					_pulse_count++;
					if(_pulse_count < pulses.duration1)
					{
						if(_pwm_slope < MAX_PWM_ALL)
						{
							_pwm_slope += ((MAX_PWM_ALL / pulses.duration1) + 1);
							TIM1->CCR3 = _pwm_slope; // Close Event
						}
						//// TEST HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
					}
					else
					{
						TIM1->CCR3 = 0; // Close Event
						_pwm_slope = 0;
						trip1 = 1;
						//// TEST HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

						//// TEST HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
						//// TEST HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
					}
				}
				else
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
					////TEST HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
				}

				if(millis == pulses.stone2)
				{
					_pulse_count = 0;
				}
			}
		}
}

void enableTriggerOut(char *buff)
{
	if(buff[0] == '1')
	{
		flag_pulse_out = 1;
	}

	if(buff[0] == '2')
	{
		flag_pulse_out = 2;
	}

	if(buff[0] == '3')
	{
		flag_send_bufferA = 1;
	}
}

void ManualSequence(char *buff)
{
	if(buff[0] == '3')
	{

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

  uint32_t a_shot = 0;
  uint32_t b_shot = 0;
  uint32_t lidxA = 0;

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  	 //// TESTING
  ////HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);
  /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  //HAL_DMA_Start_IT(&hdma_adc1, SrcAddress, DstAddress, DataLength);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ad1_raw, adcChannelCount);

  HAL_TIM_Base_Start(&htim2);

  // <PWM OUTPUT>
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  TIM1->CCR1 = 0;
  b_shot = HAL_GetTick();
  HAL_Delay(1);
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;

  // <NO NEED> CDC_Init_FS();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GetTick() > (a_shot + 1000))
	  {
		  a_shot = HAL_GetTick();
		  if(adcConversionComplete == 1)
		  {
			  adcConversionComplete = 0;
			  //ad1_audio = ad1_raw[0] / 32; // map(ad1_raw[1], 0, 4096, 0, 254);
			  //sprintf(strA1, "A1:%d,Rate:%d,Map:%d\n", ad1_raw[0], conv_rate, ad1_audio);
			  sprintf(strA1, "A1:%d,Kalman:%d,Map:%d,Rate:%d\n", ad1_raw[0], kalman_adc_int, ad1_audio, conv_rate); // @suppress("Float formatting support")

			  conv_rate = 0;
			  //TIM1->CCR1 = ad1_audio;

			  if(flag_send_bufferA == 1)
			  {
				  flag_send_bufferA = 0;
				  for(lidxA=0;lidxA<128;lidxA++)
				  {
					  if(buf_num == 2)
					  {
						  sprintf(strA1, "%d,%d\n", buf1[lidxA], buf1[lidxA + 50]);
					  }
					  else
					  {
						  sprintf(strA1, "%d,%d\n", buf2[lidxA],buf2[lidxA+100]);
					  }
					  CDC_Transmit_FS(strA1, strlen(strA1));
					  HAL_Delay(20);

				  }
			  }
			  else
			  {
				  // CDC_Transmit_FS(strA1, strlen(strA1));
			  }

			  //In the video example following function is called at the end of every conversion.
			  //But my goal is to start the conversion from the trigger of the TIM2
			  //HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ad1_raw, adcChannelCount);
		  }

		  //// CDC_Transmit_FS(".", 1);
	  }

	  if(HAL_GetTick() > (b_shot + 3000))
	  {
		  b_shot = HAL_GetTick();

		  //TIM1->CCR1 = 250;
		  //HAL_Delay(10);
		  //TIM1->CCR1 = 0;

		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

		  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

		  //ad1_audio = ad1_raw[0] / 32;
		  //TIM1->CCR1 = ad1_audio;
	  }

	  if(flag_usbrx == 1)
	  {
		  // CDC_Transmit_FS(usb_rx_buffer, strlen(usb_rx_buffer));
		  flag_usbrx = 0;
		  enableTriggerOut(usb_rx_buffer);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 32;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 8;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 128;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
