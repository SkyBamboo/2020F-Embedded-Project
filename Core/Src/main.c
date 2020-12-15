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
#include "lcd.h"
#include "string.h"
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t RxBuffer1[5000];
uint8_t RxBuffer2[5000];
uint8_t message[2000];
uint16_t Front1=0, Front2=0;
uint16_t LastEnd1=0, LastEnd2=0;
uint16_t End1=0, End2=0;

int isConnected = 0;
int Mode = 0; // AP Server = 1; AP Client = 2; STA Server = 3; STA Client = 4

char Reset[] = "AT+RST\r\n";
char SetSingleConnect[] = "AT+CIPMUX=0\r\n";
char SetMultiConnect[] = "AT+CIPMUX=1\r\n";
char Server_SendDataTCP[] = "AT+CIPSEND=0,25\r\n";
char Server_SendDataUDP[] = "AT+CIPSEND=25\r\n";
char Client_SendDataTCP[] = "AT+CIPSEND=25\r\n";
char CloseConnection[] = "AT+CIPCLOSE\r\n";
// AP Mode
char SetAPMode[] = "AT+CWMODE=2\r\n";
char Default_SetWiFiInfo[] = "AT+CWSAP=\"HelloSTM32\",\"12345678\",1,4\r\n";
char Default_SetServerIP[] = "AT+CIPAP=\"192.168.0.123\"\r\n";
char Default_StartServerAndSetPort[] = "AT+CIPSERVER=1,4567\r\n";
// STA Mode
char SetSTAMode[] = "AT+CWMODE=1\r\n";
char Default_ConnectToWiFi[] = "AT+CWJAP=\"HelloSTM32\",\"12345678\"\r\n";
char Default_ConnectToTCPServer[] = "AT+CIPSTART=\"TCP\",\"192.168.0.123\",4567\r\n";
char ConnectToWiFi[] = "AT+CWJAP=\"MZ\",\"lfyz1229\"\r\n";
char ConnectToTCPServer[] = "AT+CIPSTART=\"TCP\",\"192.168.43.196\",8086\r\n";
char ConnectToUDPServer[] = "AT+CIPSTART=\"UDP\",\"ip\",port\r\n";
char SetSeriaNet[] = "AT+CIPMODE=1\r\n";
char StartSerialNet[] = "AT+CIPSEND\r\n";
char CloseSerialNet[] = "+++";

// AP & STA Mode
char SetAPSTAMode[] = "AT+CWMODE=3\r\n";

// Check List
int DefaultTimeout = 200; // true timeout = default * 10
char ResponseOK[] = "OK";
char ResponseError[] = "ERROR";
char ResponseReady[] = "ready";
char CheckConnectionStatus[] = "AT+CIPSTATUS\r\n";
char CheckLocalIP[] = "AT+CIFSR\r\n";

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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  LCD_Init();
  HAL_UART_Receive_IT(&huart1, &RxBuffer1[End1++], 1);
  HAL_UART_Receive_IT(&huart2, &RxBuffer2[End2++], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  LCD_Clear(WHITE);
//	  BACK_COLOR = WHITE;
//	  POINT_COLOR = BLACK;
//	  LCD_ShowString(10, 10, 100, 10, 24, (uint8_t *)"huart1: ");
//	  LCD_ShowString(10, 30, 100, 10, 24, (uint8_t *)RxBuffer1);
//	  LCD_ShowString(10, 50, 100, 10, 24, (uint8_t *)"huart2: ");
//	  LCD_ShowString(10, 70, 100, 10, 24, (uint8_t *)RxBuffer2);
//	  HAL_Delay(2000);



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_WK_Pin */
  GPIO_InitStruct.Pin = KEY_WK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_WK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY0_Pin */
  GPIO_InitStruct.Pin = KEY0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 2);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int SendCommand(char cmd[], char expectReponse[], int time_out)
{
	char* indexOK = 0;
	int findExpectResponse = 0;
	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
	if (expectReponse && time_out)
	{
		while(--time_out)
		{
			HAL_Delay(10);
			indexOK = strstr(RxBuffer2, expectReponse);
			if (indexOK)
			{
				findExpectResponse = 1;
				break;
			}
		}
	}

	// Check response
	if (findExpectResponse)
	{
		HAL_UART_Transmit(&huart1, "Success\r\n", 9, HAL_MAX_DELAY);

	}
	else if (time_out == 0)
	{
		HAL_UART_Transmit(&huart1, "Timeout\r\n", 9, HAL_MAX_DELAY);
	}
	else
	{
		HAL_UART_Transmit(&huart1, "Fail\r\n", 6, HAL_MAX_DELAY);
	}
	HAL_Delay(1000);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(Front1 < End1 - 1 && End1!=1)
	{
//		while(Front1 < End1-1 && End1!=1)
//		{
//			HAL_UART_Transmit(&huart1, &RxBuffer1[Front1++], 1, HAL_MAX_DELAY);
//		}
		char msg[200];
		for(int i=0;i<End1-1;i++){
			msg[i] = RxBuffer1[i];
		}
		int IsSending = 0;
		if(End1 - 1>=5 &&msg[0] == 's' && msg[1] == 'e' && msg[2] == 'n' && msg[3] == 'd' && msg[4] ==':')
			IsSending = 1;
		if(IsSending){
			char SendDataCommand[22];
			if(Mode == 1)
				sprintf(SendDataCommand, "AT+CIPSEND=0,%d\r\n", End1-1-5);
			else if(Mode == 4)
				sprintf(SendDataCommand, "AT+CIPSEND=%d\r\n", End1-1-5);
			HAL_UART_Transmit(&huart2, SendDataCommand, 22, HAL_MAX_DELAY); //send command to wifi

			HAL_UART_Transmit(&huart1, &msg[5], End1-1-5, HAL_MAX_DELAY);	//message echo
			HAL_UART_Transmit(&huart2, &msg[5], End1-1-5, HAL_MAX_DELAY);	//send message to wifi
		} else {
			msg[End1-1] = '\r';
			msg[End1] = '\n';
			HAL_UART_Transmit(&huart1,&msg,End1+1,HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2,&msg,End1+1,HAL_MAX_DELAY);
		}
		//reset USART1
		Front1 = 0;
		End1 = 0;
		(&huart1)->RxState = 32;
		HAL_UART_Receive_IT(&huart1, &RxBuffer1[End1++], 1);
	}

	while(Front2 < End2 - 1 && End2!=1)
	{
		HAL_UART_Transmit(&huart1, &RxBuffer2[Front2++], 1, HAL_MAX_DELAY);
	}
	if(Front2 >= End2 - 1 && End2 != 1)
	{
		Front2 = 0;
		End2 = 0;
		(&huart2)->RxState = 32;
		HAL_UART_Receive_IT(&huart2, &RxBuffer2[End2++], 1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		HAL_UART_Receive_IT(&huart1, &RxBuffer1[End1++], 1);
	}
	if (huart == &huart2)
	{
		HAL_UART_Receive_IT(&huart2, &RxBuffer2[End2++], 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
	case KEY0_Pin:
		if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET) {
			HAL_UART_Transmit(&huart1, "SetSTA\r\n", 8, HAL_MAX_DELAY);

			SendCommand(SetSTAMode, ResponseOK, DefaultTimeout);
			SendCommand(Reset, ResponseOK, DefaultTimeout);
			HAL_Delay(1000);
			HAL_Delay(1000);
			SendCommand(Default_ConnectToWiFi, ResponseOK, DefaultTimeout);
			SendCommand(SetSingleConnect, ResponseOK, DefaultTimeout);
			SendCommand(Default_ConnectToTCPServer, ResponseOK, DefaultTimeout);
			//SendCommand(SetSeriaNet, ResponseOK, DefaultTimeout);
			//SendCommand(StartSerialNet, ResponseOK, DefaultTimeout);
			Mode = 4;
			HAL_UART_Transmit(&huart1, "Finish\r\n", 8, HAL_MAX_DELAY);
		}
		break;
	case KEY1_Pin:
		if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
			HAL_UART_Transmit(&huart1, "SetAP\r\n", 7, HAL_MAX_DELAY);

			SendCommand(SetAPMode, ResponseOK, DefaultTimeout);
			SendCommand(Reset, ResponseOK, DefaultTimeout);
			HAL_Delay(1000);
			HAL_Delay(1000);
		    SendCommand(Default_SetWiFiInfo, ResponseOK, DefaultTimeout);
			SendCommand(Default_SetServerIP, ResponseOK, DefaultTimeout);
			SendCommand(SetMultiConnect, ResponseOK, DefaultTimeout);
			SendCommand(Default_StartServerAndSetPort, ResponseOK, DefaultTimeout);
			Mode = 1;
			HAL_UART_Transmit(&huart1, "Finish\r\n", 8, HAL_MAX_DELAY);
		}
		break;
	case KEY_WK_Pin:
		if (HAL_GPIO_ReadPin(KEY_WK_GPIO_Port, KEY_WK_Pin) == GPIO_PIN_SET) {
			//SendCommand(CloseSerialNet, ResponseOK, DefaultTimeout);
			SendCommand(CloseConnection, ResponseOK, DefaultTimeout);
		}
		break;
	default:
		break;
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
