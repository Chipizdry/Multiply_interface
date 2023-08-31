/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"
#include "stdint.h"
#include "stdio.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define LED1_ON     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,GPIO_PIN_SET);
#define LED1_OFF    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,GPIO_PIN_RESET);

#define LED2_ON     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_SET);
#define LED2_OFF    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_RESET);

#define REL1_2_ON     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
#define REL1_2_OFF    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);

#define REL2_1_ON     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
#define REL2_1_OFF    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);

#define REL2_2_ON     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);
#define REL2_2_OFF    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);

#define REL3_1_ON     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET);
#define REL3_1_OFF    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET);

#define REL1_1_ON     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);
#define REL1_1_OFF    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET);

#define REL3_2_ON     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_SET);
#define REL3_2_OFF    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET);

#define OWR_ON     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
#define OWR_OFF    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);

#define ISOL_ON     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,GPIO_PIN_SET);
#define ISOL_OFF    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,GPIO_PIN_RESET);

//#define   BTN  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0;
//#define   INP  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9);

#define SETTINGS_ADDRESS 0x08007C00
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
#define VDD_CALIB ((uint32_t) (3300))
#define VDD_APPLI ((uint32_t) (3000))
#define AVG_SLOPE  .0025
#define V25  0.76

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
//uint16_t adc_val[7]={0,};
uint16_t adc_out=0;
uint16_t period=0;
uint16_t period_x=0;
uint16_t pulse=0;
uint16_t pulse_x=0;
uint16_t v_batt=0;
uint16_t v_CPU=0;
uint16_t temp=0;
uint16_t inp_1=0;
uint16_t inp_2=0;
uint16_t inp_3=0;
uint16_t inp_4=0;
uint8_t alarm=0;
char str[38]={0, };
uint8_t rcvd[248]={0,};
uint8_t count=0;
uint8_t rcv_addres=0;
uint8_t addres_call=0;
uint8_t directive=0;
uint8_t addres=3;
uint8_t new_addres=0;
uint8_t Device_ID=157;//108 БСА
uint8_t temp_ID=0;
uint8_t settings[4]={0,};
uint8_t line_status=0;
uint8_t isol_status=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */


uint32_t FlashRead(uint32_t address) {
 return (*(__IO uint32_t*)address);
}



void WriteConfig() {
 HAL_FLASH_Unlock(); // Открыть доступ к FLASH (она закрыта от случайной записи)
 // В структуре settings хранятся настройки, преобразую ее в 16-битный массив для удобства доступа
 uint16_t* data = (uint16_t*) &settings;
 FLASH_EraseInitTypeDef ef; // Объявляю структуру, необходимую для функции стирания страницы
 HAL_StatusTypeDef stat;
 ef.TypeErase = FLASH_TYPEERASE_PAGES; // Стирать постранично
 ef.PageAddress = SETTINGS_ADDRESS; // Адрес страницы для стирания
 ef.NbPages = 1; //Число страниц = 1
 uint32_t temp; // Временная переменная для результата стирания (не использую)
 HAL_FLASHEx_Erase(&ef, &temp); // Вызов функции стирания
 // Будьте уверены, что размер структуры настроек кратен 2 байтам
 for (int i = 0; i < sizeof(settings); i += 2) { // Запись всех настроек
  stat = HAL_FLASH_Program (FLASH_TYPEPROGRAM_HALFWORD, SETTINGS_ADDRESS + i, *(data++));
  if (stat != HAL_OK) break; // Если что-то пошло не так - выскочить из цикла
 }
 HAL_FLASH_Lock(); // Закрыть флешку от случайной записи
}



// Пример чтения только 4 байт настроек. Для бОльшего объема данных используйте цикл
void ReadConfig() {
 // Структуру настроек превращаю в указатель на массив 8-ми битных значений
 uint8_t* setData = (uint8_t*)&settings;
 LED1_ON;

 uint32_t tempData = FlashRead(SETTINGS_ADDRESS); // Прочесть слово из флешки
// uint32_t tempData  = *(__IO uint32_t *)SETTINGS_ADDRESS;
 if (tempData != 0xffffffff) { // Если флешка не пустая
 setData[0] = (uint8_t)((tempData & 0xff000000) >> 24); // �?звлечь первый байт из слова
 setData[1] = (uint8_t)((tempData & 0x00ff0000) >> 16); // �?звлечь второй байт из слова
 setData[2] = (uint8_t)((tempData & 0x0000ff00) >> 8); // �?злечь третий байт из слова
 setData[3] = tempData & 0xff; // �?звлечь четвертый байт из слова
 }
}






void Print_test(void){

	//sprintf(str, "\n\r Cnt-%02d,per-%04d,pul-%04d,adr-%04d,dir-%04d\n\r",count,period,pulse,rcv_addres,directive);
	sprintf(str, "\n\r Cnt-%02d,pul-%04d,adr-%04d,dir-%04d\n\r",count,rcvd[count],rcv_addres,directive);
	// LED2_ON;
	 HAL_UART_Transmit_DMA(&huart1, str, sizeof(str));

}

void TCT(void){

	                 if (count==13){OWR_ON;}
	                 if ((count==14)&&(alarm==1)){OWR_ON;}
		        	 if (count==22){OWR_ON;}
		        	 if (count==31){OWR_ON;}
		        	 if (count==40){OWR_ON;}
		        	 if (count==49){OWR_ON;}
		        	 if (count==58){OWR_ON;}
		        	 if (count==67){OWR_ON;}
		        	 if (count==76){OWR_ON;}
		        	 if (count==85){OWR_ON;}
		        	 if (count==94){OWR_ON;}
		        	 if (count==103){OWR_ON;}
		        	 if (count==112){OWR_ON;}
		        	 if (count==121){OWR_ON;}
		        	 if (count==130){OWR_ON;}
		        	 if (count==139){OWR_ON;}
		        	 if (count==148){OWR_ON;}
		        	 if (count==157){OWR_ON;}

}




void Protocol(void){
	           //  if ((count==9)&&(alarm=1)){OWR_ON;}

              TCT();

	        	 if (count==13)
	        	         	   {  directive=0;

	        	         	     directive|= (rcvd[9]<<4)|(rcvd[10]<<3)|(rcvd[11]<<2)|(rcvd[12]<<1)|(rcvd[13]) ;

	        	         	   OWR_ON;
	        	         	    }


	        	         	 switch(directive)
	        	         			   {

	        	         	case 0 :
                                  TCT();
                                  if (count==14){OWR_ON;}

	        	            break;

	        	         	case 1 :
	        	         	      TCT();
	        	         	     if (count==14){OWR_ON;}
	        	            break;

	        	         	 case 2 :
	        	         		        	         		 TCT();

	        	         		        	         		if(count==14){OWR_ON;}
	        	         		        	         		if(count==15){OWR_ON;}

	        	         		        	        //  if((count>=14)&&(count<22))
	        	         		        	         //    {
	        	         		        	         //		 temp_ID|=((Device_ID>>(21-count))&(0b1));
	        	         		        	         //		 if(temp_ID==1){OWR_ON;}
	        	         		        	         //		 if(temp_ID==0){OWR_OFF;}
	        	         		        	         //		     temp_ID=0;
	        	         		        	         //		       }

	        	         		        	         	 break;

	        	          	 case 3 :

	        	         	                                   TCT();

	        	         		        	         		 if((count>=14)&&(count<22))
	        	         		        	         		        	         	       {
	        	         		        	         		        	         	            temp_ID|=((Device_ID>>(21-count))&(0b1));
	        	         		        	         		        	         	            if(temp_ID==1){OWR_ON;}
	        	         		        	         		        	         	            if(temp_ID==0){OWR_OFF;}
	        	         		        	         		        	         	            temp_ID=0;
	        	         		        	         		        	         	       }



	        	         		        	         	     if((count>=23)&&(count<31)&&(alarm==1))  //Статус прибора.выходов
	        	         		        	         	             	        	      			{
	        	         		        	         	             	        	      				temp_ID|=((1)&(0b1));
	        	         		        	         	             	        	      				if(temp_ID==1){ OWR_ON;}
	        	         		        	         	             	        	      				if(temp_ID==0){ }
	        	         		        	         	             	        	      				temp_ID=0;
	        	         		        	         	            	        	      			}

	        	         		        	         	     if((count>=32)&&(count<40)){OWR_ON;}
	        	         		        	         	   if((count>=41)&&(count<49)){OWR_ON;}

	        	         		        	         	 //     if((count>=42)&&(count<49)&&(alarm==1))  //Статус прибора.выходов
	        	         		        	         	   //   			{
	        	         		        	         	   //   				temp_ID|=((1)&(0b1));
	        	         		        	         	   //   				if(temp_ID==1){ OWR_ON;}
	        	         		        	         	    //  				if(temp_ID==0){ }
	        	         		        	         	    //  				temp_ID=0;
	        	         		        	         	     // 			}
	        	         		        	              //Тестовый опрос
	        	         		        	         	//      if(count==44){ OWR_ON;}




	        	         		        	         	  break;


	        	         	case 4 :
	        	         		      TCT();
	        	         		break;



	        	         	 case 6 :

	        	         		 if(count==30){OWR_ON;}
	        	         	  if(count==45){OWR_ON;}
	        	            if(count==46)
	        	           {
	        	            	if((rcvd[35]==1)&&(rcvd[42]==1)) {LED1_ON;}
	        	                if((rcvd[36]==1)&&(rcvd[43]==1)) {LED1_OFF;}
	        	                if((rcvd[37]==1)&&(rcvd[44]==1)) {LED2_ON;}
	        	                if((rcvd[38]==1)&&(rcvd[45]==1)) {LED2_OFF;}

	        	         			        	         			   }


	        	         	 break;

	        	         	 case 13 :

	        	         		        	         		 if(count==30){OWR_ON;}
	        	         		        	         	  if(count==45){OWR_ON;}
	        	         		        	         	 if(count==54){OWR_ON;}
	        	         		        	         	 if(count==63){OWR_ON;}
	        	         		        	         	 if(count==72){OWR_ON;}
	        	         		        	         	 if(count==81){OWR_ON;}
	        	         		        	         	 if(count==90){OWR_ON;}
	        	         		        	         	 if(count==99){OWR_ON;}
	        	         		        	         	 if(count==108){OWR_ON;}
	        	         		        	         	 if(count==117){OWR_ON;}
	        	         		        	         	 if(count==126){OWR_ON;WriteConfig();}
	        	         		        	         	//if(count==127){OWR_ON;WriteConfig();}

	        	         		        	            if(count==46)
	        	         		        	           {
	        	         		        	            	if((rcvd[35]==1)&&(rcvd[42]==1)) {LED1_ON;}
	        	         		        	                if((rcvd[36]==1)&&(rcvd[43]==1)) {LED1_OFF;}
	        	         		        	                if((rcvd[37]==1)&&(rcvd[44]==1)) {LED2_ON;}
	        	         		        	                if((rcvd[38]==1)&&(rcvd[45]==1)) {LED2_OFF;}

	        	         		        	         			        	         			   }

	        	         		        	           if (count==54)
	        	         		        	                           	        	    {
	        	         		        	                           	        	      new_addres=0;
	        	         		        	                           	        	      new_addres|= (rcvd[47]<<7)|(rcvd[48]<<6)|(rcvd[49]<<5)|(rcvd[50]<<4)|(rcvd[51]<<3)|(rcvd[52]<<2)|(rcvd[53]<<1)|(rcvd[54]);
                                                                                          addres=new_addres;
                                                                                          settings[0]=new_addres;
                                                                                          WriteConfig();
                                                                                          HAL_ResumeTick();
	        	         		        	                           	        	    }




	        	         		        	         	 break;



	        	         	  case 14 :

	        	         			   if(count==30){OWR_ON;}
	        	         			   if(count==45){OWR_ON;}
	        	         			   if(count==54){OWR_ON;}

	        	         			   if(count==46)
	        	         			   {

	        	         				   if((rcvd[35]==1)&&(rcvd[42]==1)) {LED1_ON;}
	        	         				   if((rcvd[36]==1)&&(rcvd[43]==1)) {LED1_OFF;}
	        	         				   if((rcvd[37]==1)&&(rcvd[44]==1)) {LED2_ON;}
	        	         				   if((rcvd[38]==1)&&(rcvd[45]==1)) {LED2_OFF;}
	        	         			   }
	        	         			   break;



	        	         			   }
                                  }




void ADC_reset_ch(void){

	 ADC_ChannelConfTypeDef sConfig = {0};

	              sConfig.Channel = ADC_CHANNEL_0;
	              sConfig.Rank = ADC_RANK_NONE;
	              HAL_ADC_ConfigChannel(&hadc, &sConfig);
	              sConfig.Channel = ADC_CHANNEL_1;
	              sConfig.Rank = ADC_RANK_NONE;
	              HAL_ADC_ConfigChannel(&hadc, &sConfig);
	              sConfig.Channel = ADC_CHANNEL_4;
	              sConfig.Rank = ADC_RANK_NONE;
	              HAL_ADC_ConfigChannel(&hadc, &sConfig);
	              sConfig.Channel = ADC_CHANNEL_5;
	              sConfig.Rank = ADC_RANK_NONE;
	              HAL_ADC_ConfigChannel(&hadc, &sConfig);
	              sConfig.Channel = ADC_CHANNEL_8;
	              sConfig.Rank = ADC_RANK_NONE;
	              HAL_ADC_ConfigChannel(&hadc, &sConfig);
	              sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	              sConfig.Rank = ADC_RANK_NONE;
	              HAL_ADC_ConfigChannel(&hadc, &sConfig);
	              sConfig.Channel = ADC_CHANNEL_VREFINT;
	              sConfig.Rank = ADC_RANK_NONE;
	              HAL_ADC_ConfigChannel(&hadc, &sConfig);



}
void ADC_Select_CH0 (void)
	  {
	      ADC_ChannelConfTypeDef sConfig = {0};

	  	  sConfig.Channel = ADC_CHANNEL_0;
	  	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  	 HAL_ADC_ConfigChannel(&hadc, &sConfig);

	  }

	  void ADC_Select_CH1 (void)
	    {
	    	ADC_ChannelConfTypeDef sConfig = {0};
	    	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	    	  */
	    	  sConfig.Channel = ADC_CHANNEL_1;
	    	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;;
	    	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	    	  HAL_ADC_ConfigChannel(&hadc, &sConfig);

	    }


	  void ADC_Select_CH4 (void)
	     {
	     	ADC_ChannelConfTypeDef sConfig = {0};
	     	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	     	  */
	     	  sConfig.Channel = ADC_CHANNEL_4;
	     	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	     	  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	     	  HAL_ADC_ConfigChannel(&hadc, &sConfig);
	     }


	  void ADC_Select_CH5 (void)
	       {
	       	ADC_ChannelConfTypeDef sConfig = {0};
	       	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	       	  */
	       	  sConfig.Channel = ADC_CHANNEL_5;
	       	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	       	  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	       	  HAL_ADC_ConfigChannel(&hadc, &sConfig);
	       }


	  void ADC_Select_CH8 (void)
	         {
	         	ADC_ChannelConfTypeDef sConfig = {0};
	         	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	         	  */
	         	  sConfig.Channel = ADC_CHANNEL_8;
	         	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	         	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	         	  HAL_ADC_ConfigChannel(&hadc, &sConfig);
	         }

	  void ADC_Select_CHTemp (void)
	  {
	  	ADC_ChannelConfTypeDef sConfig = {0};
	  	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  	  */
	  	  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	  	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  	  sConfig.SamplingTime =ADC_SAMPLETIME_28CYCLES_5;
	      HAL_ADC_ConfigChannel(&hadc, &sConfig);
	  }


	  void ADC_Select_CH_V_REF (void)
	    {
	    	ADC_ChannelConfTypeDef sConfig = {0};
	    	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	    	  */
	    	  sConfig.Channel = ADC_CHANNEL_VREFINT;
	    	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	    	  sConfig.SamplingTime =ADC_SAMPLETIME_13CYCLES_5;
	    	  HAL_ADC_ConfigChannel(&hadc, &sConfig);
	    }

	  uint16_t ADC_read(uint8_t n)

	  {

		  switch (n){

			  case 0:
		  ADC_reset_ch();
          ADC_Select_CH0();
          break;
			  case 1:
				  ADC_reset_ch();
		  ADC_Select_CH1();
          break;

			  case 2:
				  ADC_reset_ch();
		  ADC_Select_CH4();
		  break;

			  case 3:
				  ADC_reset_ch();
		  ADC_Select_CH5();
		  break;

			  case 4:
				  ADC_reset_ch();
		  ADC_Select_CH8();
		  break;

			  case 5:
				  ADC_reset_ch();
		  ADC_Select_CHTemp();
		  break;

			  case 6:
				  ADC_reset_ch();
		  ADC_Select_CH_V_REF();
		  break;

		  }
 		  HAL_ADC_Start(&hadc);
 		  HAL_ADC_PollForConversion(&hadc, 100);
 		  adc_out = HAL_ADC_GetValue(&hadc);
 		  HAL_ADC_Stop(&hadc);
 		// adc_val[n]=adc_out;
 		// return adc_val[n];

 		  return adc_out;
	  }
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  ReadConfig();

      ISOL_ON;
      alarm=1;

     HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
     HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
     HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
     HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);


     addres=settings[0];
     if(settings[0]==0xFF){addres=0;}
     LED1_OFF;
     LED2_OFF;
     HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_SuspendTick();
   HAL_PWR_EnableSleepOnExit ();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	//  LED1_ON;
	//  WriteConfig();
	//  LED1_OFF;
	//  HAL_SuspendTick();
	 // HAL_PWR_EnableSleepOnExit ();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */


  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 499;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 40000;
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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICFilter = 10;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  huart1.Init.BaudRate = 256000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ISOL_Pin|OWR_Pin|LED_1_Pin|LED_2_Pin
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT_1_1_Pin|OUT_1_2_Pin|REL_2_Pin|REL_3_Pin
                          |REL_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ISOL_Pin LED_1_Pin LED_2_Pin PA15 */
  GPIO_InitStruct.Pin = ISOL_Pin|LED_1_Pin|LED_2_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OWR_Pin */
  GPIO_InitStruct.Pin = OWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_1_Pin */
  GPIO_InitStruct.Pin = BTN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_1_1_Pin OUT_1_2_Pin REL_2_Pin REL_3_Pin
                           REL_4_Pin */
  GPIO_InitStruct.Pin = OUT_1_1_Pin|OUT_1_2_Pin|REL_2_Pin|REL_3_Pin
                          |REL_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
 {
     if (htim->Instance == TIM1)
     {


         if((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)&&(line_status==0))
         {

        	 period = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);


        	 TIM1->CNT=0;
         if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0)   {alarm=1;}
        	 OWR_OFF;
        	 if((count==192)||(period>=100)){

        		 count=0;}

        	 line_status=1;
        	 }


         if((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)&&(line_status==0))
                  {
                 	 period_x = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);

                 	 TIM1->CNT=0;
                  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0)   {alarm=1;}
                 	 OWR_OFF;
                 	 if((count==192)||(period_x>=100)){

                 		 count=0;

                 	 }
                 	 ISOL_ON;
                 	 line_status=2;
                 	 }


         HAL_PWR_EnableSleepOnExit ();
         }



          if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // FALLING с HIGH на LOW
                        { pulse = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
                          HAL_IWDG_Refresh(&hiwdg);

                          if(line_status==1){ISOL_OFF};
                          line_status=0;
                        OWR_OFF;

             if((pulse>55)&&(pulse<60))  {count=0;rcvd[count]=2;rcv_addres=0;directive=0;}
        	 if((pulse>36)&&(pulse<40))  rcvd[count]=1;
        	 if((pulse>16)&&(pulse<21))  rcvd[count]=0;
        	// Print_test();
        	 if((rcvd[1]==1)&&(rcvd[2]==0)&&(count==2))
        	        	                        	  {
        	        	                        		 addres_call=addres_call+1;
        	        	                        		 if(addres_call==addres)
        	        	                        		    {OWR_ON;}}

        	 if (count==8)
        	        	    {
        	        	      rcv_addres=0;
        	        	      rcv_addres|= (rcvd[1]<<7)|(rcvd[2]<<6)|(rcvd[3]<<5)|(rcvd[4]<<4)|(rcvd[5]<<3)|(rcvd[6]<<2)|(rcvd[7]<<1)|(rcvd[8]);
        	        	      if(rcv_addres==0){addres_call=0;}

        	        	    }
        	 if((rcv_addres==addres)&&(count>8)){Protocol();}


        	  count++;

                        }



          if((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)&&(line_status!=0)) // FALLING с HIGH на LOW
                                 {
                                   pulse_x = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);
                                   HAL_IWDG_Refresh(&hiwdg);

                                   line_status=0;
                                 OWR_OFF;

                      if((pulse_x>55)&&(pulse_x<60))  {count=0;rcvd[count]=2;rcv_addres=0;directive=0;}
                 	 if((pulse_x>36)&&(pulse_x<40))  rcvd[count]=1;
                 	 if((pulse_x>16)&&(pulse_x<21))  rcvd[count]=0;

                 	 if((rcvd[1]==1)&&(rcvd[2]==0)&&(count==2))
                 	        	                        	  {
                 	        	                        		 addres_call=addres_call+1;
                 	        	                        		 if(addres_call==addres)
                 	        	                        		    {OWR_ON;}}

                 	 if (count==8)
                 	        	    {
                 	        	      rcv_addres=0;
                 	        	      rcv_addres|= (rcvd[1]<<7)|(rcvd[2]<<6)|(rcvd[3]<<5)|(rcvd[4]<<4)|(rcvd[5]<<3)|(rcvd[6]<<2)|(rcvd[7]<<1)|(rcvd[8]);
                 	        	      if(rcv_addres==0){addres_call=0;}

                 	        	    }
                 	 if((rcv_addres==addres)&&(count>8)){Protocol();}


                 	  count++;
                                 }











          HAL_PWR_EnableSleepOnExit ();
     }


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)

{
  LED2_OFF;
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
