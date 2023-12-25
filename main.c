/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "lcd16x2_i2c.h"
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
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
//Question N°2
                    // «C», «d» ,«H» ,«U», «1»
uint32_t Tamb,TAB[5]={0xc6,0xa1,0x89,0xc1,0xf9};

/* USER CODE BEGIN PV */
typedef enum {OFF=0, ON} etat;
etat etat_syst;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
uint32_t ConvLM35(void);
void Test_AudioVisuel(void);
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
    /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  
  etat_syst =  OFF; 
 while (1)
  {
    /* USER CODE END WHILE */
    lcd16x2_i2c_printf("BONJOUR ULT 2023");
		HAL_Delay(200);
		lcd16x2_i2c_2ndLine();
	   lcd16x2_i2c_printf("Temp = %d %c", 25,'C');
		HAL_Delay(200);
   //**************************************************
	// Test du Boutton Start  
while( HAL_GPIO_ReadPin(GPIOA,Start_Pin) ==0); 
  	etat_syst = !etat_syst;
			do{  // Traitement de la climatiation 
Tamb = ConvLM35();	
lcd16x2_i2c_2ndLine();
	   lcd16x2_i2c_printf("Temp = %d",Tamb);
		HAL_Delay(1000);				
//ii.	Chauffage /Refroidissement OFF.
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_5,GPIO_PIN_RESET);
Test_AudioVisuel();
 				
// Si (Tamb < 15°C) on , et  
 if (Tamb < 204) 
 { //active le chauffage (ON)
 	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5,GPIO_PIN_RESET);
 //on allume la LED Bleue 
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2,GPIO_PIN_RESET); 
	 //on affiche la lettre « C » sur un afficheur
  GPIOB->ODR= TAB[0]; 
	 HAL_Delay(1000); 
 }
 
 
/* Si (Tamb > 35°C)*/ 
if (Tamb > 478)
{ //on active le Refroidissement (ON)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5,GPIO_PIN_SET);
	//on allume la LED Rouge
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1,GPIO_PIN_RESET);
  //on affiche la lettre  « H » 
GPIOB->ODR= TAB[2]; 
	 HAL_Delay(1000); 
 //on génère une alarme sonore à travers le Buzzer
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_SET);
	HAL_Delay(2000); 
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_RESET);
}
 
/* Si  (15 =< Tamb <= 35) */ 
if ((Tamb >= 204) && (Tamb <= 478))  
{//on allume la LED Verte
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_2,GPIO_PIN_RESET);
	//on affiche la valeur 1 
GPIOB->ODR= TAB[4]; 
	 HAL_Delay(1000); 
//on désactive le chauffage et le Refroidissement
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3|GPIO_PIN_5,GPIO_PIN_RESET);
}	
			}while( etat_syst== ON);	
			
  }  // Fin While(1)
}// Fin main
uint32_t ConvLM35() 
{  uint32_t val_convertie;
// start ADC entree Analogique = PA2
while (HAL_ADC_Start(&hadc2)!= HAL_OK);

	//attendre la fin de conversion ou Timeout = 200 ms
while(HAL_ADC_PollForConversion(&hadc2,200)!=HAL_OK);

	// lire la valeur convertie
	val_convertie = HAL_ADC_GetValue(&hadc2);
		
	while( HAL_ADC_Stop(&hadc2)!= HAL_OK);
return val_convertie;
}			 
//*********************************** 
void Test_AudioVisuel(void)
{
//iii.	Test Visuel des LEDs 
// 8 Leds du AFF 7SEG et LED_B, LED_R et LED_V
	GPIOB->ODR = 0x0; //Allumer les LEDS de AFF 7SEG
  HAL_Delay(1000); 
  GPIOB->ODR=0xff; //Eteindre les LEDS de AFF 7SEG
	
     GPIOC->ODR =0x07; // Allumer LED_R,LED_V et LED_B
				HAL_Delay(1000); 
          GPIOC->ODR =	0;// Eteindre LED_R,LED_V et LED_B			
	//Test Audio du Buzzer.
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); 
		HAL_Delay(2000); 		
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
}
//****************
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_B_Pin|LED_V_Pin|LED_R_Pin|CHAUFFAGE_Pin
                          |BUZZER_Pin|REFROIDISSEMENT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_B_Pin LED_V_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin|LED_V_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CHAUFFAGE_Pin REFROIDISSEMENT_Pin */
  GPIO_InitStruct.Pin = CHAUFFAGE_Pin|REFROIDISSEMENT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Start_Pin */
  GPIO_InitStruct.Pin = Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UP_Temp_Pin Down_Temp_Pin */
  GPIO_InitStruct.Pin = UP_Temp_Pin|Down_Temp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
