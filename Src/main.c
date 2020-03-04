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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Fuel combustion
float inputAmoutPerImpuls  = 0.5336;      // ml per each impuls
float outputAmoutPerImpuls  = 0.5336;      // ml per each impuls

// Received information
int totalInputImpuls;           // impulsy na wejsciu
int totalOutputImpuls;          // impulsy na powrocie
int countAverage = 0; 			// 0 - not count, 1 - count

// Current combustion
unsigned long previousTime; 	// ms
int totalInputImpulsNew;         // impulsy na wejsciu
int totalOutputImpulsNew;       // impulsy na powrocie
float previousCombustion;       // L/H

// Display
float totalCombustion;          // L
float currentCombustion;        // L/H

// Wheel gauge
// Received information
int machineWidth;				// cm
int distancePerImpuls; 			// m

int totalWheelImpuls;
unsigned long totalWheelImpulsNew;
unsigned long previousTime;

// Display
float area;						// ha
float velocity;					// km/h
// Temperature
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t sum, RH, TEMP;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void countTotalFuel();
void countCurrentFuel(int differenceTimeMs);
void countArea();
void countVelocity(int differenceTimeMs);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void delay_us (uint16_t us);
void DHT11_Start (void);
void readDHT();
void sendData();
void Set_DHT_Pin_Output(void);
void Set_DHT_Pin_Input(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// TIM1 - send data through bluetooth - interrupt 300ms
	// TIM2 - calculate current fuel combustion and velocity - interrupt 100ms
	if(htim->Instance == TIM1) {
		uart_write(&huart2, "tekst");
	}
	if(htim->Instance == TIM2) {
		countCurrentFuel(100);
		countVelocity(100);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == GPIO_PIN_6) {

	}
	if(GPIO_Pin == GPIO_PIN_7) {
		totalInputImpuls++;
	}
	if(GPIO_Pin == GPIO_PIN_0) {
		totalOutputImpuls++;
	}
	if(GPIO_Pin == GPIO_PIN_1) {
		totalWheelImpuls++;
	}


}

void uart_write(UART_HandleTypeDef * handler, char * text) {
       HAL_UART_Transmit(handler, text, strlen(text), 1000);
}

void uart_write_line(UART_HandleTypeDef * handler, char * text) {

       HAL_UART_Transmit(handler, text, strlen(text), 1000);
       HAL_UART_Transmit(handler, "\r\n", 2, 1000);
}

void uart_read_line(UART_HandleTypeDef * handler, char * buffer, uint16_t buffer_size) {
       HAL_StatusTypeDef status;
       char current_char;
       uint16_t char_counter = 0;

       while (char_counter < buffer_size - 1) {
             status = HAL_UART_Receive(handler, &current_char, 1, 1);
             if (status == HAL_OK) {
                    if (current_char == '\r' || current_char == '\n')
                           if (char_counter == 0) continue; else break;
                    *(buffer + char_counter++) = current_char;
             }
       }

       *(buffer + char_counter) = '\0';
}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim3,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim3) < us);  // wait for the counter to reach the us input in the parameter
}

void DHT11_Start (void)
{
	//Set_Pin_Output(DHT11_GPIO_Port, DHT11_Pin);  // set the pin as output
	Set_DHT_Pin_Output();
	HAL_GPIO_WritePin (DHT11_GPIO_Port, DHT11_Pin, 0);   // pull the pin low
	delay_us(18000);   // wait for 18ms
	Set_DHT_Pin_Input();
	//Set_Pin_Input(DHT11_GPIO_Port, DHT11_Pin);    // set as input
}

void Set_DHT_Pin_Output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure GPIO pin output: PA9 */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

void Set_DHT_Pin_Input(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure GPIO pin input: PA9 */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}
uint8_t Check_Response (void)
{
	uint8_t Response = 0;
	delay_us(40);
	if (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)))
	{
		delay_us(80);
		if ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin))) Response = 1;
		else Response = -1;
	}
	while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)));   // wait for the pin to go high
		delay_us(40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)));  // wait for the pin to go low
	}
	return i;
}

void readDHT() {
	DHT11_Start();
	Check_Response();
	Rh_byte1 = DHT11_Read();
	Rh_byte2 = DHT11_Read();
	Temp_byte1 = DHT11_Read();
	Temp_byte2 = DHT11_Read();
	sum = DHT11_Read();
	HAL_Delay(500);
}

void countTotalFuel() {
	float totalInputFuel = totalInputImpuls * inputAmoutPerImpuls; 		// ml
	float totalOutputFuel = totalOutputImpuls * outputAmoutPerImpuls;	// ml
	totalCombustion = (totalInputFuel - totalOutputFuel) / 1000;		// L
}

void countCurrentFuel(int differenceTimeMs) {
	float differenceInputFuel   = (totalInputImpulsNew-totalInputImpuls) * inputAmoutPerImpuls; 	// ml
	float differenceOutputFuel  = (totalOutputImpulsNew-totalOutputImpuls) * outputAmoutPerImpuls;	// ml
	float differenceTime = differenceTimeMs/3600000;	// hour
	currentCombustion = ( (differenceInputFuel-differenceOutputFuel)/1000) / differenceTime;
}

void countArea() {
	float widthPerImpuls = machineWidth * distancePerImpuls;  // cm * m
	widthPerImpuls /= 100 * 10000; // convert to ha
	area = widthPerImpuls * totalWheelImpuls;
}

void countVelocity(int differenceTimeMs) {
	int wheelDistanceDifference = ((totalWheelImpulsNew - totalWheelImpuls) * distancePerImpuls)/1000; // km
	float differenceTime = differenceTimeMs/3600000;	// hour
	velocity = wheelDistanceDifference/differenceTime;
}

void sendData() {
	float totalCombustion;          // L
	float currentCombustion;        // L/H
	float area;						// ha
	float velocity;					// km/h
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  countArea();
	  countTotalFuel();
	  readDHT();
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  htim1.Init.Prescaler = 10000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 240;
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
  htim2.Init.Prescaler = 10000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 80;
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
  htim3.Init.Prescaler = 8-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff-1;
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : inputFlowMeter_Pin */
  GPIO_InitStruct.Pin = inputFlowMeter_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(inputFlowMeter_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : outputFlowMeter_Pin wheelGauge_Pin */
  GPIO_InitStruct.Pin = outputFlowMeter_Pin|wheelGauge_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
