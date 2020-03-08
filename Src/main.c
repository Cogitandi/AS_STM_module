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
#include <stdio.h>
#include <stdlib.h>
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Bluetooth
int bluetoothReceivedArrayLenght = 10;
uint8_t data[50]; // Tablica przechowujaca wysylana wiadomosc.
uint16_t size = 0; // Rozmiar wysylanej wiadomosci
uint8_t Received[10];


// settings
int countImpulses = 0;

// Fuel combustion
float inputAmoutPerImpuls  = 0.5336;      // ml per each impuls
float outputAmoutPerImpuls  = 0.5336;      // ml per each impuls

// Received information
int totalInputImpuls;           // impulsy na wejsciu
int totalOutputImpuls;          // impulsy na powrocie
int countAverage = 0; 			// 0 - not count, 1 - count

// Current combustion
unsigned long previousTime; 	// ms
int totalInputImpulsPrevious;         // impulsy na wejsciu
int totalOutputImpulsPrevious;       // impulsy na powrocie
float previousCombustion;       // L/H

// Display
float totalCombustion;          // L
float currentCombustion;        // L/H

// Wheel gauge
// Received information
int machineWidth;				// cm
float distancePerImpuls; 			// m

int totalWheelImpuls;
unsigned long totalWheelImpulsPrevious;
unsigned long previousTime;

// Display
float area;						// ha
float velocity;					// km/h
// Temperature
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void countTotalFuel();
void countCurrentFuel(int differenceTimeMs);
void countArea();
void countVelocity(int differenceTimeMs);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void delay_us (uint16_t us);
void sendData();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
int fetchValue(char letter, uint8_t data[]);

void delay_us (uint16_t us);
void readDHT();
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT22_Start (void);
uint8_t DHT22_Check_Response (void);
uint8_t DHT22_Read (void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//uart_write(&huart2, "tekst");
	// TIM1 - send data through bluetooth - interrupt 300ms
	// TIM2 - calculate current fuel combustion and velocity - interrupt 100ms
	if(htim->Instance == TIM1) {
		countCurrentFuel(300);
		countVelocity(300);
		countArea();
		countTotalFuel();
		sendData();

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
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

/*********************************** DHT22 FUNCTIONS ****************************************/

#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_6

void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim3,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim3) < us);  // wait for the counter to reach the us input in the parameter
}

void readDHT() {
	DHT22_Start();
	Presence = DHT22_Check_Response();
	Rh_byte1 = DHT22_Read ();
	Rh_byte2 = DHT22_Read ();
	Temp_byte1 = DHT22_Read ();
	Temp_byte2 = DHT22_Read ();
	SUM = DHT22_Read();

	TEMP = ((Temp_byte1<<8)|Temp_byte2);
	RH = ((Rh_byte1<<8)|Rh_byte2);

	Temperature = (float) (TEMP/10.0);
	Humidity = (float) (RH/10.0);
	HAL_Delay(1000);
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT22_Start (void)
{
	Set_Pin_Output(DHT22_PORT, DHT22_PIN); // set the pin as output
	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin low
	delay_us (1200);  // wait for > 1ms
	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);   // pull the pin high
	delay_us (20);   // wait for 30u
	Set_Pin_Input(DHT22_PORT, DHT22_PIN);   // set as input
}

uint8_t DHT22_Check_Response (void)
{
	uint8_t Response = 0;
	delay_us (40);  // wait for 40us
	if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) // if the pin is low
	{
		delay_us (80);   // wait for 80us

		if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))){
			Response = 1;  // if the pin is high, response is ok
		} else {
			Response = -1;
		}
	}

	while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));   // wait for the pin to go low
	return Response;
}

uint8_t DHT22_Read (void)
{
	uint8_t i = 0;
	uint8_t j = 0;

	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));   // wait for the pin to go high
		delay_us (40);   // wait for 40 us

		if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));  // wait for the pin to go low
	}

	return i;
}

/*********************************** Fuel,Area FUNCTIONS ****************************************/

void countTotalFuel() {
	float totalInputFuel = totalInputImpuls * inputAmoutPerImpuls; 		// ml
	float totalOutputFuel = totalOutputImpuls * outputAmoutPerImpuls;	// ml
	totalCombustion = (totalInputFuel - totalOutputFuel) / 1000;		// L
}

void countCurrentFuel(int differenceTimeMs) {
	float differenceInputFuel   = (totalInputImpuls-totalInputImpulsPrevious) * inputAmoutPerImpuls; 	// ml
	float differenceOutputFuel  = (totalOutputImpuls-totalOutputImpulsPrevious) * outputAmoutPerImpuls;	// ml
	float differenceTime = differenceTimeMs/3600000;	// hour
	currentCombustion = ( (differenceInputFuel-differenceOutputFuel)/1000) / differenceTime;
	totalInputImpulsPrevious = totalInputImpuls;
	totalOutputImpulsPrevious = totalOutputImpuls;
}

void countArea() {
	float widthPerImpuls = machineWidth * distancePerImpuls;  // cm * m
	widthPerImpuls /= 100 * 10000; // convert to ha
	area = widthPerImpuls * totalWheelImpuls;
}

void countVelocity(int differenceTimeMs) {
	int wheelDistanceDifference = ((totalWheelImpuls - totalWheelImpulsPrevious) * distancePerImpuls)/1000; // km
	float differenceTime = differenceTimeMs/3600000;	// hour
	velocity = wheelDistanceDifference/differenceTime;
	totalWheelImpulsPrevious = totalWheelImpuls;
}

void sendData() {
	/*
	float totalCombustion;          // L
	float currentCombustion;        // L/H
	float area;						// ha
	float velocity;					// km/h
	*/

	if(countImpulses) {
		size = sprintf(data, "%d\r\n", totalWheelImpuls); // Stworzenie wiadomosci do wyslania oraz przypisanie ilosci wysylanych znakow do zmiennej size.
		HAL_UART_Transmit_IT(&huart2,data,size);
	} else {
		// TODO: check width and impluses != 0
		if(machineWidth && distancePerImpuls ) {
			int a = totalCombustion*100;
			int b = currentCombustion*100;
			int c = area*100;
			int d = velocity*100;
			int e = Temperature*100;
			int f = Humidity*100;
			size = sprintf(data, "%d,%d,%d,%d,%d,%d\r\n", a,b,c,d,e,f); // Stworzenie wiadomosci do wyslania oraz przypisanie ilosci wysylanych znakow do zmiennej size.
			HAL_UART_Transmit_IT(&huart2,data,size);
		}
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// A - start counting impulses
	// B - stop counting impulses
	// C - set machine width
	// D - set impulses for 100m


 for(int i=0;i<bluetoothReceivedArrayLenght;i++) {
	 switch(Received[i]) {
	 case 'A':
		 totalWheelImpuls = 0;
		 countImpulses = 1;
		 break;
	 case 'B':
	 	 countImpulses = 0;
	 	 break;
	 case 'C':
		 machineWidth = fetchValue('C',Received);
		 break;
	 case 'D':
		 distancePerImpuls = fetchValue('D',Received)/100;
		 break;
	 }

 }


 size = sprintf(data, "Odebrana wiadomosc: %s\n\r",Received);
 HAL_UART_Transmit_IT(&huart2, data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
 HAL_UART_Receive_IT(&huart2, Received, 10); // Ponowne w³¹czenie nas³uchiwania
}

int fetchValue(char letter, uint8_t data[]) {
	char value[10];

	for(int i=0;i<bluetoothReceivedArrayLenght;i++) {
		if(data[i] == letter) {
			int j=i+1;
			int l=0;
			while(data[j] != ',') {
				if(j==bluetoothReceivedArrayLenght) break;
				value[l++] = data[j++];
			}
			return atoi(value);
		}
	}
	return 0;
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start_IT(&htim1);
//HAL_TIM_Base_Start_IT(&htim2);
HAL_TIM_Base_Start(&htim3);
HAL_UART_Receive_IT(&huart2, Received, 10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
  {
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
  HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DHT22_Pin */
  GPIO_InitStruct.Pin = DHT22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);

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
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
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
