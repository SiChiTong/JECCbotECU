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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIDAR_STATE_INIT 0
#define LIDAR_STATE_SCAN 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char lidarReceiveBuffer[7];

char kvhReceiveChar;

char gpsReceiveChar;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void powertrainInit();

void powertrainEnableMotors(int enableState);

void powertrainSetSpeeds(int16_t left, int16_t right);

void lidarInit();
void lidarStart();
void lidarDecode();

void kvhInit();
void kvhStart();
void kvhDecode();

void gpsInit();
void gpsStart();
void gpsDecode();

void insertSensordataToApi();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		kvhDecode();
	}
	else if(huart->Instance == huart2.Instance)
	{
		lidarDecode();
	}
	else if(huart->Instance == huart3.Instance)
	{
		gpsDecode();
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
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  apiInit();

  powertrainInit();

  lidarInit();

  kvhInit();

  gpsInit();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  insertSensordataToApi();

	  ApiInstruction ins = apiUpdate();
	  if(ins.responseLen > 0)
	  {
		  CDC_Transmit_FS(ins.response, ins.responseLen);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0x7fff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.Init.BaudRate = 4800;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, INH34_Pin|INH12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_ONBOARD_Pin */
  GPIO_InitStruct.Pin = LED_ONBOARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ONBOARD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INH34_Pin INH12_Pin */
  GPIO_InitStruct.Pin = INH34_Pin|INH12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void powertrainInit()
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	powertrainEnableMotors(1);

	powertrainSetSpeeds(0, 0);
}


void powertrainEnableMotors(int enableState)
{
	HAL_GPIO_WritePin(INH12_GPIO_Port, INH12_Pin, enableState);
	HAL_GPIO_WritePin(INH34_GPIO_Port, INH34_Pin, enableState);
}

void powertrainSetSpeeds(int16_t left, int16_t right)
{
	if(left < 0)
	{
		left = -left;
		htim3.Instance->CCR3 = 0;
		htim3.Instance->CCR4 = left;
	}
	else
	{
		htim3.Instance->CCR3 = left;
		htim3.Instance->CCR4 = 0;
	}

	if(right < 0)
	{
		right = -right;
		htim3.Instance->CCR1 = 0;
		htim3.Instance->CCR2 = right;
	}
	else
	{
		htim3.Instance->CCR1 = right;
		htim3.Instance->CCR2 = 0;
	}
}

void lidarInit()
{
	char lidarInitStr[2] = { 0xa5, 0x20 };

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_UART_Transmit(&huart2, lidarInitStr, 2, 1000);
	HAL_UART_Receive_IT(&huart2, lidarReceiveBuffer, 7);

	lidarStart();
}

void lidarStart()
{
	lidarData.available = false;

	HAL_UART_Receive_IT(&huart2, lidarReceiveBuffer, 5);
}

void lidarDecode()
{
		static int lidarState = LIDAR_STATE_INIT;

		static int frameCounter = 0;

		int quality, s, sinv, c, angle, distance;

		if(lidarState == LIDAR_STATE_INIT)
		{
			if(0xa5 == lidarReceiveBuffer[0] && 0x5a == lidarReceiveBuffer[1] && 0x05 == lidarReceiveBuffer[2] && 0x00 == lidarReceiveBuffer[3] && 0x00 == lidarReceiveBuffer[4] && 0x40 == lidarReceiveBuffer[5] && 0x81 == lidarReceiveBuffer[6])
			{
				lidarState = LIDAR_STATE_SCAN;
//				HAL_UART_Receive_IT(&huart2, lidarReceiveBuffer, 5);
			}
			else
			{
				lidarState = LIDAR_STATE_INIT;
				lidarInit();
			}
		}
		else if(lidarState == LIDAR_STATE_SCAN)
		{
			memcpy(lidarData.frameBuffer[frameCounter], lidarReceiveBuffer, 5);

			frameCounter ++;
			if(frameCounter >= LIDAR_BUFFERSIZE)
			{
				lidarData.available = true;
				frameCounter = 0;
			}

			if(!lidarData.available)
			{
				HAL_UART_Receive_IT(&huart2, lidarReceiveBuffer, 5);
			}
		}
}

void kvhInit()
{
	  char kvhInitStr[3] = { 's', '\r' };
	  char kvhConfigNmeaStr[5] = { '=', 't' , ',', '0', '\r' };
	  char kvhConfigSpeedStr[7] = { '=', 'r', ',' , '6', '0', '0', '\r' };
	  char kvhConfigUnitStr[5] = { '=', 'i' , ',', 'd', '\r' };
	  HAL_UART_Transmit(&huart1, kvhInitStr, 2, 1000);
	  HAL_Delay(100);
	  HAL_UART_Transmit(&huart1, kvhConfigNmeaStr, 5, 1000);
	  HAL_Delay(100);
	  HAL_UART_Transmit(&huart1, kvhConfigUnitStr, 5, 1000);
	  HAL_Delay(100);
	  HAL_UART_Transmit(&huart1, kvhConfigSpeedStr, 7, 1000);

	  kvhStart();
}

void kvhStart()
{
	kvhData.available = false;

	HAL_UART_Receive_IT(&huart1, &kvhReceiveChar, 1);
}

void kvhDecode()
{
	static int cursor = 0;
	static char nmeaString[20];

	if(kvhReceiveChar == '$')
	{
		cursor = 0;
	}
	else if(kvhReceiveChar == '\r')
	{
		nmeaString[cursor] = '\0';
		if(strncmp("$HCHDT", nmeaString, 6) == 0)
		{
			char headingStr[4];
			strncpy(headingStr, &nmeaString[7], 3);
			headingStr[3] = '\0';

			int heading = strtol(headingStr, NULL, 10);
			if(heading > 180)
			{
				heading -= 360;
			}
			kvhData.heading = heading;
			kvhData.available = true;
		}
	}
	else
	{
		cursor ++;
	}

	nmeaString[cursor] = kvhReceiveChar;
	if(!kvhData.available)
	{
		HAL_UART_Receive_IT(&huart1, &kvhReceiveChar, 1);
	}
}

void gpsInit()
{
	gpsStart();
}

void gpsStart()
{
	gpsData.available = false;

	HAL_UART_Receive_IT(&huart3, &gpsReceiveChar, 1);
}

void gpsDecode()
{
	static int cursor = 0;
	static char nmeaString[100];

	if(kvhReceiveChar == '$')
	{
		cursor = 0;
	}
	else if(kvhReceiveChar == '\n')
	{
		nmeaString[cursor] = '\0';
		strcpy(nmeaString, "$GNRMC,174752.00,A,4900.06642,N,01249.68048,E,0.102,,110420,,,A*6E\n\0");
		if(strncmp("$GPRMC", nmeaString, 6) == 0 || strncmp("$GNRMC", nmeaString, 6) == 0)
		{
			int fieldIndex=0;
			int charIndex=0;

			char fields[13][15];
			char currentField[15];

			float lat, lon;

			for(int i=7; i<strlen(nmeaString); i++)
			{
			  char currentChar=nmeaString[i];
			  if(currentChar!=',')
			  {
			    currentField[charIndex]=currentChar;
			    charIndex++;
			  }
			  else
			  {
				currentField[charIndex] = '\0';
			    strcpy(fields[fieldIndex], currentField);
			    charIndex = 0;
			    fieldIndex++;
			    if(fieldIndex > 5)
			    {
			    	break;
			    }
			  }
			}

			//decode time -> fieldIndex 0
			char timeStr[7];
			uint32_t time;
			strncpy(timeStr, fields[0], 6);
			timeStr[6] = '\0';
			time = strtol(timeStr, NULL, 10);

				//decode latitude -> fieldIndex 2/3
				char ddLat[3];
				char mmLat[8];
				strncpy(ddLat, fields[2], 2);
				ddLat[2] = '\0';
				strncpy(mmLat, &fields[2][2], 7);
				mmLat[7] = '\0';

				lat=atof(ddLat) + atof(mmLat)/60;
				if(fields[3][0]=='S')
				  lat=-lat;


	//			//decode longitude -> fieldIndex 4/5
				char ddLon[4];
				char mmLon[8];
				strncpy(ddLon, fields[4], 3);
				ddLon[3] = '\0';
				strncpy(mmLon, &fields[4][3], 7);
				mmLon[7] = '\0';

				lon=atof(ddLon) + atof(mmLon)/60;
				if(fields[6][0]=='W')
				  lon=-lon;

			gpsData.time = time;
			gpsData.lat = lat;
			gpsData.lon = lon;
			gpsData.available = true;
		}
	}
	else
	{
		cursor ++;
	}

	nmeaString[cursor] = gpsReceiveChar;
	if(!gpsData.available)
	{
		HAL_UART_Receive_IT(&huart3, &gpsReceiveChar, 1);
	}
}

void insertSensordataToApi()
{
	if(kvhData.available)
	{
		apiWrite16(API_REG_HEADING_KVH, kvhData.heading);
		kvhStart();
	}

	if(gpsData.available)
	{
		apiWrite32(API_BENCH_GPS_START, gpsData.time);
		apiWriteFloat(API_BENCH_GPS_START + 2, gpsData.lat);
		apiWriteFloat(API_BENCH_GPS_START + 4, gpsData.lon);
		gpsStart();
	}

	if(lidarData.available)
	{
		for(int i = 0; i < LIDAR_BUFFERSIZE; i++)
		{
			int s = lidarData.frameBuffer[i][0] & 0b00000001;
			int sinv = (lidarData.frameBuffer[i][0] & 0b00000010) >> 1;
			int quality = (lidarData.frameBuffer[i][0] & 0b11111100) >> 2;
			int c = lidarData.frameBuffer[i][1] & 0b00000001;
			if(s == !sinv && c ==1)
			{
	//			if(s == 1)
	//			{
	//				data flush according to datasheet ignored
	//			}
				if(quality > 0)
				{
					uint16_t angle = lidarData.frameBuffer[i][2];
					angle = angle << 8;
					int tmp = lidarData.frameBuffer[i][1] & 0b11111110;
					tmp = tmp >> 1;
					angle += tmp;
					angle = angle / 64;
					while(angle > 360)
					{
						angle -= 360;
					}
					uint16_t distance = lidarData.frameBuffer[i][4];
					distance = distance << 8;
					distance += lidarData.frameBuffer[i][3];
					distance = distance / 4;

					apiWrite16(API_BENCH_LIDAR_START + angle, distance);
				}
			}
			else
			{
				lidarInit();
			}
		}
		lidarStart();
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
