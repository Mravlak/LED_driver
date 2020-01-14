/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "matrix.h"

/*
#define LED_NUM_ROW 6
#define LED_NUM_COLUMN 10
#define NUM_SCAN_BYTES 12
#define NUM_ROW_IN_LED_MATRIX 6
#define NUM_COLUMN_IN_LED_MATRIX 10
#define SPI_TRANSFER_SIZE 1
#define Matrix_number 15
*/

uint8_t frameOut[LED_NUM_ROW][LED_NUM_COLUMN] = {0};
uint8_t ledPattern[NUM_SCAN_BYTES];
int l=0;

int PWM=0;
int PWM_DEL=0;
int PWM_FADE=1;
int BRIGHTNESS =0;
int SPI_Delay=5;
static int InterruptCounter;

int test[3][2] = {  
   {0, 1} ,   
   {4, 5} ,   
   {8, 9}   
};

int TEST[3][3] = { 
         {1, 2, 3},
         {4, 5, 6},
         {7, 8, 9}
                 };
  
  int TEST_EMPTY[3][3] = { 0 };

  int *ptrTEST = &TEST[0][0];
  int *ptrTEST_EMPTY = &TEST_EMPTY[0][0];





//uint8_t (*p2)[Matrix_number][LED_NUM_ROW][LED_NUM_COLUMN]= &LED_MAP1;
uint8_t (*p2)[Matrix_number][LED_NUM_ROW][LED_NUM_COLUMN];




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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);

void LEDCORE_BitmapToLEDPattern(uint8_t Bitmap[Matrix_number][LED_NUM_ROW][LED_NUM_COLUMN],
                                uint8_t Out[NUM_SCAN_BYTES])
{
  //#define POS_A
#ifdef POS_A
  uint8_t i, j;
  uint8_t firstByte = 1, secondByte = 0xff;
  
  for(i = 0; i < NUM_ROW_IN_LED_MATRIX; i++) {
    firstByte = (1 << i);
    secondByte = 0;
    for (j = 0; j < 2; j++) {
      firstByte = firstByte | (Bitmap [BRIGHTNESS][i][j] << (6 + j));
    }
    Out[2*i] = firstByte;
    for (j = 2; j < NUM_COLUMN_IN_LED_MATRIX ; j++) {
      secondByte = (secondByte ) |  (Bitmap [BRIGHTNESS][i][j] << (j - 2));
    }
    Out[2*i + 1] = secondByte;
  }
#endif /*POS_A*/
  
#define POS_B
#ifdef POS_B
  int8_t i, j;
  uint8_t firstByte, secondByte;
  
  for(i = 0; i < NUM_ROW_IN_LED_MATRIX; i++) {
    firstByte = 0x0;//(1 << (NUM_ROW_IN_LED_MATRIX -1 - i));
    secondByte = 0x0;
    for(j = 0; j < 2; j++) {
      firstByte = firstByte | (Bitmap [BRIGHTNESS][i][j] << (6 + j));
    }                   
    
   
    
    for(j = 2; j < NUM_COLUMN_IN_LED_MATRIX ; j++) {
      secondByte = (secondByte ) |  (Bitmap [BRIGHTNESS][i][j] << (j - 2));
    }
    
    if ((firstByte) || (secondByte))
      firstByte = firstByte | (1 << (NUM_ROW_IN_LED_MATRIX -1 - i));
    
    Out[(2*NUM_ROW_IN_LED_MATRIX)-(2*i)-2] = firstByte;          
    Out[(2*NUM_ROW_IN_LED_MATRIX)-(2*i)-1] = secondByte;
  }
#endif /*POS_B*/
}
int32_t LEDCORE_DrawLEDPattern(uint8_t *Buffer, uint8_t CurIndex)
{
  
  uint8_t tempBuf[2];
  uint8_t prevIndex;
#define SPI_DELAY 5
  
  if (CurIndex == 0)
    prevIndex = 2 * (NUM_ROW_IN_LED_MATRIX - 1);
  else
    prevIndex = CurIndex - 2;
    
  if (CurIndex > 2 * (NUM_ROW_IN_LED_MATRIX - 1))
  {
    while(1) 
    {
    }
  }
 tempBuf[0] = 0xc0 & Buffer[prevIndex++];
 tempBuf[1] = 0xff & Buffer[prevIndex];

  
  /*send WA data*/
  
  /* Make LE line low*/
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    
  /* Start a SPI transfer*/
  if (HAL_SPI_Transmit(&hspi2, tempBuf, SPI_TRANSFER_SIZE, SPI_DELAY) != HAL_OK)
  {
    return -1;
  }
 
  /* Provide LE pulse*/
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  {
    /*
      volatile uint32_t delay;
      int i;
      for(i=0;i<1000;i++){
        delay++;
      } */
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
 /* 
  if (!DimmingDelayCounter)
  {
     __HAL_TIM_SET_COMPARE(&TIMERHandle1,TIM_CHANNEL_1, 0);
     HAL_TIM_PWM_Start(&TIMERHandle1, TIM_CHANNEL_1);
  }
  */
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
htim1.Instance->CCR1 = PWM;
 

  /*send actual data*/
  // HAL_Delay(1);
  tempBuf[0] = Buffer[CurIndex++];
  tempBuf[1] = Buffer[CurIndex];
   
  /* Start a SPI transfer*/
  if (HAL_SPI_Transmit(&hspi2, tempBuf, SPI_TRANSFER_SIZE, SPI_DELAY) != HAL_OK)
  {
      return -1;
  }
  
    /* Provide LE pulse*/
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  
  
      
  return 0;
}
/* USER CODE BEGIN PFP */

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
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
//ZAGON TIMER1
HAL_TIM_Base_Start_IT(&htim1);
//ZAGON TIMER 3
HAL_TIM_Base_Start_IT(&htim3);
// ZAGON PWM
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//htim1.Instance->CCR1 = 600;

//HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
HAL_Delay(1);
l=l+1;
if(l<2000)
{
  p2=&LED_MAP2;
}
  if((l>2000)&(l<4000))
{
 p2=&LED_MAP1;

}
if((l>4000)&(l<6000))
{
   p2=&LED_MAP3;
}
if(l>6000)
{
 l=0;
}


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 
  }
  /* USER CODE END 3 */
}

void SPI_SEND()
{
  LEDCORE_BitmapToLEDPattern(*p2, ledPattern);
   LEDCORE_DrawLEDPattern(&ledPattern[0], (2 * InterruptCounter));
  
  InterruptCounter++;
 
  if (InterruptCounter == NUM_ROW_IN_LED_MATRIX)
  {
    InterruptCounter = 0;
     BRIGHTNESS++;
     if(BRIGHTNESS>(Matrix_number-1))
       BRIGHTNESS=0;
  PWM_DEL=PWM_DEL++;
  if(PWM_DEL>2)
  {
 PWM=PWM+PWM_FADE;
   if((PWM==0)||(PWM==620))
   {
     PWM_FADE=-PWM_FADE;
   }
   PWM_DEL=0;
  }
    
  }

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
__HAL_SPI_ENABLE(&hspi2);
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

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1; 
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 640;
  
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  //htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
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
  sConfigOC.Pulse = 320;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
 htim3.Init.Prescaler =199;
 //htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39;
//htim3.Init.Period = 99;
//htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.ClockDivision = 0;
  //htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim3)
{
  SPI_SEND();
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
