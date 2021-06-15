/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h> 
#include <string.h>
#include  <stdio.h>
#include <complex.h>
#include <stdbool.h>
 
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

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static const uint16_t AHT10_ADDR = 0x38 << 1;
static const uint8_t AHT10_SOFT = 0xBA;

static const uint16_t ADXL345_ADDR = 0x53 << 1;

static const uint16_t CCS811_ADDR = 0x5A << 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
//AHT10 FUNCS
static void AHT_Init(void);
static void AHT_Measure(void);
static uint32_t AHT_Humidity(void);
static uint32_t AHT_Temperature(void);

//ADXL345 FUNCS
static void ADXL_Init(float* m);
static float ADXL_Measure(float* m);
float ADXL_Calculate();
void ADXL_Transform(float);

//FFT FUNCS
uint8_t expon(uint16_t value);
void swap(float *x, float *y);
void compute(float *vReal, float *vImag, uint16_t samples, uint8_t power);

//MAX9814 FUNCS
void MAX_Init(void);
void MAX_Measure(void);

//CCS811 FUNCS
void CCS_Init(void);
void CCS_Measure(void);
bool CCS_Ready(void);
bool CCS_Error(void);
bool CCS_FWMode(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Global Variables
uint8_t data_aht[6];
uint8_t data_adxl[6];
uint8_t data_ccs[8];
uint32_t data_mic;
float acc;
uint8_t to_transfer[18];

uint16_t led_counter = 0;
uint16_t event_counter = 1;


float acc_mean[3];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//General Local Variables
	
	//AHT10 Related Local Variables
	uint8_t buf[18];
	HAL_StatusTypeDef ret;
	//ADXL345 Related Local Variables
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	AHT_Init();
 	ADXL_Init(acc_mean);
	HAL_ADC_Start(&hadc1);
	CCS_Init();
	MAX_Init();
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_Delay(50);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_Delay(50);

	while (1)
  {
		// AHT10 MEASUREMENT BEGINS HERE -----------------------

		AHT_Measure();
		AHT_Humidity();
		AHT_Temperature();

		// AHT10 MEASUREMENT ENDS HERE -------------------------

		// CCS811 MEASUREMENT STARTS HERE ---------------------
		
		CCS_Measure();

		// CCS811 MEASUREMENT STARTS HERE ----------------------
		
		// MAX9814 MEASUREMENT STARTS HERE ---------------------
		
		MAX_Measure();
		
		// MAX9814 MEASUREMENT ENDS HERE -----------------------
		
		// ADXL345 MEASUREMENT BEGINS HERE 

		acc = ADXL_Calculate();
		ADXL_Transform(acc);
		
		// ADXL345 MEASUREMENT ENDS HERE -----------------------

		//UART COMM. WITH ESP32 BEGINS HERE --------------------

		// data_aht -> 6 Byte,
		// freq -> 4 Byte (32-bit)
		// Mic -> 4 Byte (32-bit)
		// CO2 -> 2 Byte (16-bit)
		// VTOC -> 2 Byte (16-bit)
		
		HAL_GPIO_TogglePin(Led_t_GPIO_Port,Led_t_Pin);
		HAL_UART_Transmit(&huart2, to_transfer, 18, HAL_MAX_DELAY);
		HAL_Delay(200);
		HAL_GPIO_TogglePin(Led_t_GPIO_Port,Led_t_Pin);
		

		//UART COMM. WITH ESP32 ENDS HERE ----------------------

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	
  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CCS_Reset_GPIO_Port, CCS_Reset_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Led_Pin|Led_t_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CCS_Reset_Pin */
  GPIO_InitStruct.Pin = CCS_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CCS_Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Led_Pin Led_t_Pin */
  GPIO_InitStruct.Pin = Led_Pin|Led_t_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if (!HAL_GPIO_ReadPin(Led_GPIO_Port, Led_Pin))
		{
			HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
			led_counter = 0;
		} else {
			if ( led_counter == 1)
			{					
				HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
				led_counter = 0;
			} else {
				led_counter++;
			}	
		}
	}
	else	if (htim->Instance == TIM3)
	{
		if (event_counter % 900 == 0)
		{
				HAL_GPIO_TogglePin(CCS_Reset_GPIO_Port, CCS_Reset_Pin);		
				HAL_GPIO_TogglePin(CCS_Reset_GPIO_Port, CCS_Reset_Pin);			
		}
		event_counter++;
	}
}


// Functions Related to AHT10
void AHT_Init(void)
{
		uint8_t com[3];
		com[0] = AHT10_SOFT;	
		HAL_I2C_Master_Transmit(&hi2c2, AHT10_ADDR, com, 1, HAL_MAX_DELAY);					

		com[0] = 0xE1;
		com[1] = 0x08;
		com[2] = 0x00;		
		HAL_I2C_Master_Transmit(&hi2c2, AHT10_ADDR, com, 3, HAL_MAX_DELAY);
}


void AHT_Measure(void)
{
	uint8_t com[3];
	uint8_t buf[20];

	com[0] = 0xAC;
	com[1] = 0x33;
	com[2] = 0;
	HAL_I2C_Master_Transmit(&hi2c2, AHT10_ADDR, com, 3, HAL_MAX_DELAY);
	
	HAL_Delay(100);
	HAL_I2C_Master_Receive(&hi2c2, AHT10_ADDR, data_aht, 6, HAL_MAX_DELAY);
	
	uint16_t idx = 0;
	for (int i = 0; i < 6; i++)
	{
		to_transfer[idx + i] = data_aht[i];
	}
			
}	


uint32_t AHT_Humidity(void)   
{	
		uint8_t buf[20];
	
		uint32_t humid_c = data_aht[1];
		humid_c <<= 8;
		humid_c |= data_aht[2];
		humid_c <<= 4;
		humid_c |= data_aht[3] >> 4;
	
		sprintf((char*)buf,
				"Humidity: %u,%u Percent\r\n",
				((int)humid_c * 100 / 0x100000),
				((int)humid_c % 100));
				
		HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
		HAL_Delay(50);		
				
		return humid_c;
}


uint32_t AHT_Temperature(void)   
{	
		uint8_t buf[20];
	
		uint32_t temp_c = data_aht[3] & 0x0F;
		temp_c <<= 8;
		temp_c |= data_aht[4];
		temp_c <<= 8;
		temp_c |= data_aht[5];
		sprintf((char*)buf,
						"Temperature: %u,%u\r\n",
						(((int)temp_c * 200 / 0x100000) - 50),
						((int)temp_c % 100));	
						
		HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
		HAL_Delay(50);
						
		return temp_c;
}



// Functions Related to ADXL345
void ADXL_Init(float* m)
{
		int16_t x, y, z;
		float xg, yg, zg = 0;
	
		uint8_t cmd[2];
		HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, 0x00, 1, data_adxl, 1, HAL_MAX_DELAY);
		cmd[0] = 0x2D;
		cmd[1] = 0X00;
		HAL_I2C_Master_Transmit(&hi2c1, ADXL345_ADDR, cmd, 2, HAL_MAX_DELAY);
		cmd[0] = 0x2D;
		cmd[1] = 0X08;
		HAL_I2C_Master_Transmit(&hi2c1, ADXL345_ADDR, cmd, 2, HAL_MAX_DELAY);
		cmd[0] = 0x2C;
		cmd[1] = 0x0A;
		HAL_I2C_Master_Transmit(&hi2c1, ADXL345_ADDR, cmd, 2, HAL_MAX_DELAY);
		cmd[0] = 0x31;
		cmd[1] = 0X01;
		HAL_I2C_Master_Transmit(&hi2c1, ADXL345_ADDR, cmd, 2, HAL_MAX_DELAY);
	
		float sum = 0;
		for(int i = 0; i < 1024; i++)
		{
			HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, 0x32, 1, data_adxl, 6, HAL_MAX_DELAY);
			x = (data_adxl[1] << 8) | data_adxl[0];
			y = (data_adxl[3] << 8) | data_adxl[2];
			z = (data_adxl[5] << 8) | data_adxl[4];
			
			xg = xg + (x * 0.0078);
			yg = yg + (y * 0.0078);
			zg = zg + (z * 0.0078);	
		}
		
		m[0] = xg / (float)1024;
		m[1] = yg / (float)1024;
		m[2] = zg / (float)1024;
}



float ADXL_Measure(float* m)
{
		int16_t x, y, z;
		float xg, yg, zg = 0;

		HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, 0x32, 1, data_adxl, 6, HAL_MAX_DELAY);
		x = (data_adxl[1] << 8) | data_adxl[0];
		y = (data_adxl[3] << 8) | data_adxl[2];
		z = (data_adxl[5] << 8) | data_adxl[4];

		xg = x * 0.0078 - m[0];
		yg = y * 0.0078 - m[1];
		zg = z * 0.0078 - m[2];		
		
		float acc = (float)sqrt(xg*xg  + yg*yg + zg*zg); 

		return acc;
}



float ADXL_Calculate()
{
		uint16_t number_of_samples = 30;
		float data_real = 0;
		uint8_t buf[20];
	
		uint8_t cmd[2];
		cmd[0]	= 0x08;
		HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x2D, 1, cmd, 1, HAL_MAX_DELAY);	
		
		for (int i = 0; i < number_of_samples; i++)
		{
				data_real = (data_real) * 0.4 + (ADXL_Measure(acc_mean)) * 0.6;
				HAL_Delay(25);
		}
		
		cmd[0] = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x2D, 1, cmd, 1, HAL_MAX_DELAY);	

		//for debugging

		sprintf((char*)buf,
			"Acceleration: %g \r\n", data_real);						
		HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
		HAL_Delay(100);
		
		return data_real;
}



void ADXL_Transform(float f)
{
		uint8_t *array;
		array = (uint8_t*)(&f);
	
		uint16_t idx = 6;
		for (int i = 0; i < 4; i++)
		{
			to_transfer[idx + i] = array[i];
		}
}



//Functions of MAX9814

void MAX_Init()
{
		uint8_t buf[20];
		float sum = 0;
		for(int i = 0; i < 1024; i++)
		{
			HAL_ADC_PollForConversion(&hadc1, 1000);
			sum = sum + HAL_ADC_GetValue(&hadc1);
		}
		sum = sum/1024;
		
		sprintf((char*)buf,
			"Microphone Mean: %f\r\n", sum);						
		HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
		HAL_Delay(20);
		
}

void MAX_Measure(void)
{
		uint8_t buf[20];
		uint32_t mic_temp = 0;
		uint32_t mics[5];
		
		for (int i = 0; i < 5; i++)
		{
			HAL_ADC_PollForConversion(&hadc1, 1000);
			mics[i] = HAL_ADC_GetValue(&hadc1);

			if (mic_temp < mics[i])
			{
				mic_temp = mics[i];
				data_mic = mics[i];
				HAL_Delay(25);
			}
		}

		sprintf((char*)buf,
			"Microphone: %u\r\n", data_mic);						
		HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
		HAL_Delay(20);
		
		uint8_t *array;
		array = (uint8_t*)(&data_mic);
	
		uint16_t idx = 10;
		for (int i = 0; i < 4; i++)
		{
			to_transfer[idx + i] = array[i];
		}
}




// Functions of CCS811
void CCS_Init(void)
{
		uint8_t buf[20];
		HAL_StatusTypeDef ret;
		uint8_t com[4];
		com[0] = 0x11;
		com[1] = 0xE5;
		com[2] = 0x72;
		com[3] = 0x8A;
		ret = HAL_I2C_Mem_Write(&hi2c1, CCS811_ADDR, 0xFF, 1, com, 4, HAL_MAX_DELAY);
		HAL_Delay(100);
	
		ret = HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDR, 0x20, 1, data_ccs, 1, HAL_MAX_DELAY); 
		HAL_Delay(75);

		ret = HAL_I2C_Mem_Write(&hi2c1, CCS811_ADDR, 0xF4, 1, NULL, 0, HAL_MAX_DELAY);
		HAL_Delay(75);

		if (CCS_Error())
			return;
		if (!CCS_FWMode())
		{
			sprintf((char*)buf,
				"FW Mode is Low\r\n");						
			HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
			HAL_Delay(100);
			return;
		}
		// Disable interrupt and set drive mode to 1 sec
		com[0] = (0x01 << 2) | (0x00 << 3) | (0x01 << 4);
		ret = HAL_I2C_Mem_Write(&hi2c1, CCS811_ADDR, 0x01, 1, com, 1, HAL_MAX_DELAY);
		HAL_Delay(75);

		ret = HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDR, 0x24, 1, data_ccs, 1, HAL_MAX_DELAY); 
		HAL_Delay(75);

		if (CCS_Error())
		{
			sprintf((char*)buf,
				"Error Bit is High\r\n");						
			HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
			HAL_Delay(100);
			
			if (!CCS_FWMode())
			{
				sprintf((char*)buf,
					"FW Mode is Low\r\n");						
				HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
				HAL_Delay(100);
			}
			
		} else {	
			if (CCS_FWMode())
			{
				sprintf((char*)buf,
					"Reaady for Measurement\r\n");						
				HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
				HAL_Delay(100);
			}else{
				sprintf((char*)buf,
					"FW Mode is Low\r\n");						
				HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
				HAL_Delay(100);
			}
		}
}



bool CCS_Ready(void)
{
	uint8_t status;
	HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDR, 0x00, 1, data_ccs, 1, HAL_MAX_DELAY);
	status = (data_ccs[0] >> 3 & 0x01);
	
	if (status == 0x01){
		return true;
	}	else {
		return false;
	}
}



bool CCS_Error(void)
{
	uint8_t status;
	HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDR, 0x00, 1, data_ccs, 1, HAL_MAX_DELAY);
	status = (data_ccs[0] & 0x01);
	
	if (status == 0x01){
		return true;
	}	else {
		return false;
	}
}


bool CCS_FWMode(void)
{
	uint8_t status;
	HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDR, 0x00, 1, data_ccs, 1, HAL_MAX_DELAY);
	status = (data_ccs[0] >> 7 & 0x01);
	
	if (status == 0x01){
		return true;
	}	else {
		return false;
	}
}	


void CCS_Measure(void) 
	{
	uint16_t eCO2, TVOC;
	uint8_t	buf[20];
  
	if (CCS_Ready())
	{
		HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDR, 0x02, 1, data_ccs, 8, HAL_MAX_DELAY); 
		HAL_Delay(75);
	
		eCO2 = ((uint16_t)data_ccs[0] << 8) | ((uint16_t)data_ccs[1]);
		TVOC = ((uint16_t)data_ccs[2] << 8) | ((uint16_t)data_ccs[3]);
		
		uint16_t idx = 14;
		for (int i = 0; i < 4; i++)
		{
			to_transfer[idx + i] = data_ccs[i];
		}
		
		if (!CCS_Error())
		{
			sprintf((char*)buf,
				"CO2: %ippm\r\nTVOC: %i\r\n\n", eCO2, TVOC);						
			HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
			HAL_Delay(100);		
		}
		else {
			sprintf((char*)buf,
				"Error Bit is High\r\n");						
			HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
			HAL_Delay(100);
		}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
