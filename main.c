/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_microseconds(uint16_t microseconds)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0); //reset of the counter
  while ((__HAL_TIM_GET_COUNTER(&htim1)) < microseconds) {} //wait for microseconds
}

uint8_t rh_1byte, rh_2byte, temp_1byte, temp_2byte;
uint16_t sum, rh, temp;

struct Data{
 	float temperature[5];
 	float humidity[5];
 };
struct Data DHT11data;

uint8_t sensor_feedback=0;
int sensor_active=0;
int read=-1;

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_InitStruct.Pin=GPIO_Pin;
	GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_InitStruct.Pin=GPIO_Pin;
	GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull=GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
//connection
#define DHT11_DIO_PORT GPIOC
#define DHT11_PIN_DIO2 GPIO_PIN_9
#define DHT11_PIN_DIO3 GPIO_PIN_8
#define DHT11_PIN_DIO4 GPIO_PIN_7

#define DHT11_HUM_PORT GPIOB
#define DHT11_PIN_HUM1 GPIO_PIN_8
#define DHT11_PIN_HUM2 GPIO_PIN_9

void sensor_start(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{

	 HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);
	 HAL_Delay(300);

	 Set_Pin_Output(GPIOx, GPIO_Pin);
	 HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);
	 delay_microseconds(18000);
	 HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);
	 delay_microseconds(20);
	 Set_Pin_Input(GPIOx, GPIO_Pin);
}

uint8_t sensor_check_response(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	uint8_t resp=0;
	delay_microseconds(40);
	if(!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))){
		delay_microseconds(80);
		if((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))) resp=1;
		else resp=-1;
	}
	while((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))); //wait for the pin to go low

	return resp;
}

uint8_t sensor_bit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	uint8_t i,j;
	for(j=0;j<8;j++)
	{
		while(!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))); //wait for the pin to go high
		delay_microseconds(40);
		if(!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)))
		{
			i&=~(1<<(7-j)); //write 0
		}
		else i|=(1<<(7-j)); //write 1
		while((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)));
}
	return i;
}

uint8_t reset_sensor(int number){

	uint8_t feedback;
	/*
	 * number(active_sensor)-def
	 * 1-humidity1
	 * 2-humidity2
	 * 3-dio3
	 * 4-dio4
	 * 5-dio2
	 */

	switch(number){
	case 1:
		sensor_start(DHT11_HUM_PORT, DHT11_PIN_HUM1);
		feedback=sensor_check_response(DHT11_HUM_PORT, DHT11_PIN_HUM1);
		break;
	case 2:
		sensor_start(DHT11_HUM_PORT, DHT11_PIN_HUM2);
		feedback=sensor_check_response(DHT11_HUM_PORT, DHT11_PIN_HUM2);
		break;
	case 3:
		sensor_start(DHT11_DIO_PORT, DHT11_PIN_DIO3);
		feedback=sensor_check_response(DHT11_DIO_PORT, DHT11_PIN_DIO3);
		break;
	case 4:
			sensor_start(DHT11_DIO_PORT, DHT11_PIN_DIO4);
			feedback=sensor_check_response(DHT11_DIO_PORT, DHT11_PIN_DIO4);
			break;
	case 5:
			sensor_start(DHT11_DIO_PORT, DHT11_PIN_DIO2);
			feedback=sensor_check_response(DHT11_DIO_PORT, DHT11_PIN_DIO2);
			break;
	}
	return feedback;
}

void read_data(int number){
	switch(number){
		case 1:
			rh_1byte=sensor_bit(DHT11_HUM_PORT, DHT11_PIN_HUM1);
			rh_2byte=sensor_bit(DHT11_HUM_PORT, DHT11_PIN_HUM1);
			temp_1byte=sensor_bit(DHT11_HUM_PORT, DHT11_PIN_HUM1);
			temp_2byte=sensor_bit(DHT11_HUM_PORT, DHT11_PIN_HUM1);
			sum=sensor_bit(DHT11_HUM_PORT, DHT11_PIN_HUM1);
			break;
		case 2:
			rh_1byte=sensor_bit(DHT11_HUM_PORT, DHT11_PIN_HUM2);
			rh_2byte=sensor_bit(DHT11_HUM_PORT, DHT11_PIN_HUM2);
			temp_1byte=sensor_bit(DHT11_HUM_PORT, DHT11_PIN_HUM2);
			temp_2byte=sensor_bit(DHT11_HUM_PORT, DHT11_PIN_HUM2);
			sum=sensor_bit(DHT11_HUM_PORT, DHT11_PIN_HUM2);
			break;
		case 3:
			rh_1byte=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO3);
			rh_2byte=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO3);
			temp_1byte=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO3);
			temp_2byte=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO3);
			sum=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO3);
			break;
		case 4:
			rh_1byte=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO4);
			rh_2byte=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO4);
			temp_1byte=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO4);
			temp_2byte=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO4);
			sum=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO4);
			break;
		case 5:
			rh_1byte=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO2);
			rh_2byte=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO2);
			temp_1byte=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO2);
			temp_2byte=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO2);
			sum=sensor_bit(DHT11_DIO_PORT, DHT11_PIN_DIO2);
			break;
		}
				temp=temp_1byte;
				rh=rh_1byte;
				DHT11data.temperature[number-1]=(float) temp;
				DHT11data.humidity[number-1]=(float) rh;
}

void read_sensor(int number){

		  sensor_feedback=reset_sensor(number);
		  if(sensor_feedback){
			  read_data(number);
		  }
		 else {
			  	  		DHT11data.temperature[number-1]=0;
			  	  		DHT11data.humidity[number-1]=0;
			  	  	  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2) {
    read=1;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //odczyt danych z kolejnych czujnikow co 0.5s (tim2)
	  if(read){
	  		  sensor_active+=1;
	  		  if(sensor_active>5) sensor_active=1;
	  		  read_sensor(sensor_active);
	  		  read=-1;
	  	  	  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 6399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, dio4_Pin|dio3_Pin|dio2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, spi_SCLK_Pin|spi_MISO_Pin|spi_MOSI_Pin|spi_SS_Pin
                          |humidity1_Pin|humidity2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : dio4_Pin dio3_Pin dio2_Pin */
  GPIO_InitStruct.Pin = dio4_Pin|dio3_Pin|dio2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : spi_SCLK_Pin spi_MISO_Pin spi_MOSI_Pin spi_SS_Pin
                           humidity1_Pin humidity2_Pin */
  GPIO_InitStruct.Pin = spi_SCLK_Pin|spi_MISO_Pin|spi_MOSI_Pin|spi_SS_Pin
                          |humidity1_Pin|humidity2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

