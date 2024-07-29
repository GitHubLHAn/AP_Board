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
#include "socket.h"
#include "wizchip_conf.h"
#include "w5500.h"
#include "w5500_spi.h"
#include "data_process.h"

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include <Lora.h>

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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
	char TX_toPC[DATA_BUF_SIZE];
	char RX_fromPC[DATA_BUF_SIZE];
	
	uint8_t des_ip[] = {192,168,1,100};
	uint16_t des_port = 1111;
	
	LoRa myLoRa;
	char TX_toRobot[DATA_BUF_SIZE];
	char RX_fromRobot[DATA_BUF_SIZE];

	uint32_t k = 0;
	uint32_t ind = 0;
	
	uint8_t flag_rx = 0;
	uint8_t have_cmd = 0, cmd = 0;
	uint8_t res = 0;
	uint16_t flag_led = 0;
	uint16_t flag_check_sr = 0;
	uint16_t toggle = 0;
	
	uint32_t time_tick = 0;
	uint32_t start = 0, s=0;
	uint32_t end = 0, e=0;
	uint32_t time = 0, t=0;
	
	uint16_t result = 0;
	
	uint8_t auto_trans = 0;
	uint8_t check_trans = 0xCC;
	
	uint8_t result_send = 0;
	
	uint16_t num_receive = 0;
	
	uint32_t tick_debug = 0;
	
	uint16_t flag_send = 0;
	
	uint16_t dagui = 0;
	uint16_t danhan = 0;
	uint16_t gui_dung = 0;
	
	uint8_t flag_get_mode = 0;
	uint8_t check_regMode = 0x00;
	
	uint8_t flag_send_FSK = 0;
	uint8_t check_trans_FSK = 0;
	
	uint8_t check_rx = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	void my_printf(const char* format, ...) {
//    va_list args;
//    va_start(args, format);

//    char buffer[256]; 
//    vsnprintf(buffer, sizeof(buffer), format, args);

//    // Gui du lieu qua UART
//    //HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

//    va_end(args);
	}
	/*----------------------------------------------------*/
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == INT_Pin){ 
			//k++;
    }
		if(GPIO_Pin == myLoRa.DIO0_pin){
			k++;	
			flag_rx = 1;	 
		}
	}
	/*----------------------------------------------------*/
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		/* Prevent unused argument(s) compilation warning */
		UNUSED(htim);

		if (htim->Instance == htim1.Instance){ 	// every 1ms
      flag_led++;
			flag_check_sr = 1;
			if(++flag_send >= 1000){
				flag_send = 0;
				//auto_trans = 1;
			}
    }
		if (htim->Instance == htim2.Instance){ 	// every 1us
      time_tick++;
			HAL_GPIO_TogglePin(Test_point_GPIO_Port, Test_point_Pin);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
		memset(RX_fromPC, 0x00, DATA_BUF_SIZE);
		memset(TX_toPC, 0x00, DATA_BUF_SIZE);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				
		/*********************************
		 *				Init for W5500               
		 *********************************/
		w5500_Init();
		/***w5500_Config only use once when set IP and port***/
		//w5500_Config();
		
		/*----------------------------------------*/
		/***Make a Server and open a listening port***/	
		if(socket(1,Sn_MR_UDP, 1111, 0) != 1){
			my_printf("\r\n Cannot create socket!");
			toggle = 2000;
		}			
		else{
			toggle = 1000;
			my_printf("\r\nSocket Created Successdully!\r\n");
			uint8_t socket_io_mode =SOCK_IO_BLOCK;
			ctlsocket(1,CS_SET_IOMODE,&socket_io_mode);
			my_printf("Start listening on port %d \r\n",100);
			my_printf("Waiting for a client connection... \r\n");
		}
		uint8_t sr = 0x00;
		
		
		/*********************************
		 *				Init for LoRa               
		 *********************************/	
		Lora_Init(&myLoRa, &hspi2, 895);
		LoRa_reset(&myLoRa);
		result = LoRa_Config(&myLoRa);
		
		//result = FSK_Config(&myLoRa);
		
		/*Start continuous receiving*/
		LoRa_startReceiving(&myLoRa);
		//FSK_startTrasmiting(&myLoRa);

		memset(TX_toRobot, 0x00, DATA_BUF_SIZE);
		memset(RX_fromRobot, 0x00, DATA_BUF_SIZE);
		
		

		HAL_TIM_Base_Start_IT(&htim1);
		HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		tick_debug++;
		
//		sr = getSn_SR(1);		//16us
//		if(sr == 0x00){		// do not have any client
//			toggle = 1000;
//		}
//		
//		if(sr == 0x22){				// 0x17 if TCP, 0x22 if UDP
//			toggle = 100;		// have a client
//			
//			/*Receiving request/commmed from PC and transmit to robot*/
//			//have_cmd = recv(1, (uint8_t *)RX_fromPC, DATA_BUF_SIZE);		// TCP
//			
//			have_cmd = recv_udp(1, (uint8_t *)RX_fromPC, DATA_BUF_SIZE, des_ip, &des_port); 		//UDP		//35us
//			
//			
//			if(have_cmd){
//				have_cmd = 0;
//				
//				for(uint8_t i=0; i<DATA_BUF_SIZE; i++)
//					TX_toPC[i] = RX_fromPC[i];
//				sendto(1, (uint8_t *)TX_toPC, DATA_BUF_SIZE, des_ip, des_port);		//UDP		290us
				
				
//				cmd = check_RX((uint8_t*)RX_fromPC, (uint8_t*)TX_toRobot);
//						
//				if(cmd == MESS_OK){
//					/*send to robot*/
//					LoRa_transmit(&myLoRa, (uint8_t*)TX_toRobot, DATA_BUF_SIZE, 15);		// 0028s

//				}
//				else;
//					memset(RX_fromPC, 0x00, DATA_BUF_SIZE);
//					memset(TX_toPC, 0x00, DATA_BUF_SIZE);
//			}
			
//			}
		/*_______________________________________________________________________________*/		
									/*Receiving response from robot and transmit to PC*/
		/*_______________________________________________________________________________*/
			
			if(flag_rx == 1){
				flag_rx = 0;
					
				check_rx =	LoRa_receive(&myLoRa, (uint8_t*)RX_fromRobot, DATA_BUF_SIZE);		//400us
				
				if(check_rx == 12){
					danhan++;
					end = time_tick*65536 + __HAL_TIM_GetCounter(&htim2);
					time = end - start;
				}
				
				if(RX_fromRobot[DATA_BUF_SIZE-2] == TX_toRobot[DATA_BUF_SIZE-2] && RX_fromRobot[DATA_BUF_SIZE-1] == TX_toRobot[DATA_BUF_SIZE-1])
					gui_dung++;
			
//				res = check_RX((uint8_t*)RX_fromRobot, (uint8_t*)TX_toPC);
				
//				if(res == MESS_OK){
//					sendto(1, (uint8_t *)TX_toPC, DATA_BUF_SIZE, des_ip, des_port);		//UDP		290us

//				}
//				memset(RX_fromRobot, 0x00, DATA_BUF_SIZE);
//				memset(TX_toPC, 0x00, DATA_BUF_SIZE);
			}
			
			
			
		/*_______________________________________________________________________________*/
			if(flag_led >= toggle){
				flag_led = 0;
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				if(listen(1) != SOCK_OK){
					//my_printf("Cannot listen on port! %d", 1000);
					toggle = 1000;
				}
			}
			
			
			if(auto_trans == 1){
				auto_trans = 0;
				TX_toRobot[0] = 0xFF;
				for(uint8_t i=1; i< DATA_BUF_SIZE-2; i++)
					TX_toRobot[i] = i;
				
				TX_toRobot[DATA_BUF_SIZE-2] = rand() % 100;
				TX_toRobot[DATA_BUF_SIZE-1] = rand() % 100;
				
				check_trans = LoRa_transmit(&myLoRa, (uint8_t*)TX_toRobot, DATA_BUF_SIZE, 1000);
				
				if(check_trans){
					dagui++;
					start = time_tick*65536 + __HAL_TIM_GetCounter(&htim2);
				}	
				
				
				
			}
			
//			if(flag_send_FSK){
//				flag_send_FSK = 0;
//				TX_toRobot[0] = 0xFF;
//				for(uint8_t i=1; i< DATA_BUF_SIZE-2; i++)
//					TX_toRobot[i] = i;
//				
//				TX_toRobot[DATA_BUF_SIZE-2] = rand() % 100;
//				TX_toRobot[DATA_BUF_SIZE-1] = rand() % 100;
//				
//				//check_trans_FSK = FSK_transmit(&myLoRa, (uint8_t*)TX_toRobot, DATA_BUF_SIZE, 1000);
//				check_trans_FSK = LoRa_transmit(&myLoRa, (uint8_t*)TX_toRobot, DATA_BUF_SIZE, 1000);
//			}
			
			if(flag_get_mode){
				flag_get_mode = 0;
				check_regMode = LoRa_read(&myLoRa, RegOpMode);
			}
		
			
//			start = time_tick*65536 + __HAL_TIM_GetCounter(&htim2);
//			end = time_tick*65536 + __HAL_TIM_GetCounter(&htim2);
//			time = end - start;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SCS_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RESET_Pin|NSS_Pin|LED_Pin|Test_point_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCS_Pin RST_Pin */
  GPIO_InitStruct.Pin = SCS_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Pin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_Pin NSS_Pin LED_Pin */
  GPIO_InitStruct.Pin = RESET_Pin|NSS_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Test_point_Pin */
  GPIO_InitStruct.Pin = Test_point_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Test_point_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
