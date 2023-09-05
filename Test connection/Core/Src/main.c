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
#include <stdio.h>
#include <string.h>

#define rx_buffer_size 512
#define rxn_buffer_size 512

uint8_t rx_buffer[rx_buffer_size] = { 0, };
uint8_t rxn_buffer[rxn_buffer_size] = { 0, };

 uint16_t rx_buffer_len;
 uint16_t rxn_buffer_len;

 uint8_t comand=0;
 uint8_t data_len=0;
 uint8_t version=0;
 uint16_t summ=0;
 uint8_t data_point=0;
 int cnt=0;
 char buffer[512] = { 0, };
 char buffer_t[512] ={0,};
 char buffer_n[512] ={0,};
 //uint8_t tm_buffer[tm_buffer_size] = { 0, };
 uint16_t tm_buffer_len;
 uint8_t tm_buffer[]={0x55,0xaa,0x00,0x06,0x00,0x08,0x10,0x02,0x00,0x04,0x00,0x00,0x01,0x8b,0xaf};


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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART_SNIFF(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void UART_SNIFF(void){



	   for(int k=0;k<rx_buffer_len;){
		  sprintf(buffer_n, "%02x,",rx_buffer[k]);
		  HAL_UART_Transmit(&huart1,buffer_n,3,400);
		  k++;}

	for(cnt=0;cnt<rx_buffer_len;)   {

	   if((rx_buffer[cnt]==0x55)&&(rx_buffer[cnt+1]==0xAA)){
	   version=rx_buffer[cnt+2];
	   comand=rx_buffer[cnt+3];
	   data_len=rx_buffer[cnt+5];
	   data_point=rx_buffer[cnt+6];
	   summ=0;

    for(int i=cnt;i<=cnt+5+data_len;i++){
	   summ=rx_buffer[i]+summ;  }
	   summ=summ%256;

	   sprintf(buffer, "\n\rVers-%02x Cmd-%02x Data LEN-%02d DT_PT-%02x CHKS-%02x \n\r",version,comand,data_len,data_point,summ);
	   HAL_UART_Transmit(&huart1, buffer, strlen(buffer),400);




	   for(int k=cnt+6;k<=(cnt+data_len+5);){
	   sprintf(buffer_t, "Dat %02d-%02x \n\r",(k-6-cnt),rx_buffer[k]);
	   HAL_UART_Transmit(&huart1, buffer_t,12,300);
	   k++;  }

    for(int k=cnt;k<=(cnt+data_len+6);){
	   sprintf(buffer_n, "%02x, ",rx_buffer[k]);
	   HAL_UART_Transmit(&huart1,buffer_n,3,400);
	   k++; }

	   }

	   cnt++;
	}




}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

 /*
	if (huart == &huart2) { //проверка, из нужного ли uart пришли данные
				//printf("RxCpltCallback = OK\n\r");
				__HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);

			//	printf("rx_buffer is overflow\n\r");

				HAL_UART_AbortReceive_IT(&huart2);
				__HAL_UART_CLEAR_IDLEFLAG(&huart2);
				__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
				HAL_UART_Receive_IT(&huart2, rx_buffer, rx_buffer_size);

                                  	}
    */
	if (huart == &huart1) { //проверка, из нужного ли uart пришли данные
					//printf("RxCpltCallback = OK\n\r");
					__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);

				//	printf("rx_buffer is overflow\n\r");

					HAL_UART_AbortReceive_IT(&huart1);
					__HAL_UART_CLEAR_IDLEFLAG(&huart1);
					__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
					HAL_UART_Receive_IT(&huart1, rxn_buffer, rxn_buffer_size);

	                                  	}
                      }

void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart){
  /*
   if(huart == &huart2) { //проверка, из нужного ли uart пришли данные
			//printf("IDLE = OK\n\r");
			__HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
			rx_buffer_len = rx_buffer_size - huart->RxXferCount;

			//----------основные действия с входящими данными-----------
			if(huart2.gState != HAL_UART_STATE_BUSY_TX) {

				        UART_SNIFF();

			         	HAL_UART_AbortReceive_IT(&huart2);
							__HAL_UART_CLEAR_IDLEFLAG(&huart2);
							__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
							HAL_UART_Receive_IT(&huart2, rx_buffer, rx_buffer_size);
							//HAL_UART_Transmit(&huart2, tm_buffer,15,1000);
						}

		          	}
     */
   if(huart == &huart1) { //проверка, из нужного ли uart пришли данные
  			//printf("IDLE = OK\n\r");
  			__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
  			rxn_buffer_len = rxn_buffer_size - huart->RxXferCount;

  			//----------основные действия с входящими данными-----------
  			if(huart1.gState != HAL_UART_STATE_BUSY_RX) {

  				      //  UART_SNIFF();
  				HAL_UART_Transmit(&huart1, rxn_buffer,strlen(rxn_buffer),100);
  			         	HAL_UART_AbortReceive_IT(&huart1);
  							__HAL_UART_CLEAR_IDLEFLAG(&huart1);
  							__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  							HAL_UART_Receive_IT(&huart1, rxn_buffer, rxn_buffer_size);}

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
  HAL_UART_Receive_IT(&huart1, rxn_buffer, rxn_buffer_size);



  __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
 HAL_UART_Receive_IT(&huart2, rx_buffer, rx_buffer_size);






  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {







	  HAL_Delay(2000);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

