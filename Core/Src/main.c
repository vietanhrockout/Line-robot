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
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
int position;
	float Kp = 0.006;
	float Ki = 0;
	float Kd = 0.03;
	int P,I,D, lastError;
	int errors[10] = {0,0,0,0,0,0,0,0,0,0};
	int error_sum = 0;
	int last_end; // 0 left, 1 right;
	int cnt = 0;
	const int ARR = 10;
	int actives = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motor_control (double pos_left, double pos_right){
		int left_case = (pos_left < 0) ? -1 : (pos_left > 0) ? 1 : 0;
	    int right_case = (pos_right < 0) ? -1 : (pos_right > 0) ? 1 : 0;

	    switch(left_case)
	    {
	        case -1:  // pos_left < 0
	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ARR*0);
	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -ARR*pos_left);
	            break;

	        case 1:  // pos_left > 0
	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ARR*pos_left);
	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ARR*0);
	            break;

	        case 0:  // pos_left == 0
	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ARR*0);
	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ARR*0);
	            break;
	    }

	    switch(right_case)
	    {
	        case -1:  // pos_right < 0
	            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ARR*0);
	            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, -ARR*pos_right);
	            break;

	        case 1:  // pos_right > 0
	            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ARR*pos_right);
	            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ARR*0);
	            break;

	        case 0:  // pos_right == 0
	            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ARR*0);
	            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ARR*0);
	            break;
	    }
}
void sharp_turn(int act, int last_ends, int speedleft, int speedright){
//	if(act == 0){
//		cnt++;
//	}else cnt = 0;

	if(act == 0){
		switch(last_ends){
		case 1:
			motor_control(-10,40);
			break;
		case 2:
			motor_control(40,-10);
			break;
		case 3:
			motor_control(40,40);
			break;
		}
	}else motor_control(speedleft, speedright);
}
//void past_errors (int error)
//{
//  for (int i = 9; i > 0; i--)
//      errors[i] = errors[i-1];
//      errors[0] = error;
//}

int errors_sum (int index, int abs)
{
  int sum = 0;
  for (int i = 0; i < index; i++)
  {
    if (abs == 1 && errors[i] < 0)
      sum += -errors[i];
    else
      sum += errors[i];
  }
  return sum;
}
void stop(){
  motor_control(0,0); 
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  int pos = 0;
	  int active = 0;
	  GPIO_PinState Sensor1State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	  GPIO_PinState Sensor2State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	  GPIO_PinState Sensor3State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
	  GPIO_PinState Sensor4State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
	  GPIO_PinState Sensor5State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
	  GPIO_PinState Sensor6State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	  GPIO_PinState Sensor7State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);

	  if(Sensor3State == GPIO_PIN_SET){
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,1);
	  }else HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0);

	  int SensorStates[] = {Sensor1State, Sensor2State, Sensor3State, Sensor4State, Sensor5State};
	  int increments[] = {1200, 2400, 3000, 3600, 4800};

	  for (int i = 0; i < 5; i++) {
	      if (SensorStates[i] == GPIO_PIN_SET) {
	          pos += increments[i];
	          active++;

//	          if (i == 0) {  // sensor1
//	              last_end = 1;
//	          } else if (i == 4) {  // sensor5
//	              last_end = 2 ;
//	          }
	          if(i == 1 || i == 2 || i == 3){
	        	  last_end = 3;
	          }
	      }
	  }

	  if(Sensor6State == GPIO_PIN_SET){
		  last_end = 1;
		  motor_control(-10,40);
	  }
	  else if(Sensor7State == GPIO_PIN_SET){
		  last_end = 2;
		  motor_control(40,-10);

  }
//	  if(Sensor6State == GPIO_PIN_SET && active == 0){
//		  motor_control(-10,-10);
//	  }else if(Sensor7State == GPIO_PIN_SET && active == 0){
//		  motor_control(-10,-10);
//	  }
//  if(active == 5){
//    stop();
//  }
	  actives = active;
	  int position = pos/active;
	  int error = 3000 - position;
	  P = error;
	  I = errors_sum(5, 0);
	  D = error - lastError;
	  lastError = error;
	  int motorspeed = P*Kp+ I*Ki + D*Kd;
	  int speedleft= 40 - motorspeed;
	  int speedright= 40 + motorspeed;
//	  if(speedleft > 33) speedleft = 33;
//	  if(speedright > 33) speedright = 33;
//	  if(speedleft < -20) speedleft = -20;
//	  if(speedright < -20) speedright = -20;
	  sharp_turn(actives, last_end, speedleft, speedright);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  sConfigOC.Pulse = 199;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 199;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA8 PA9 PA10 PA11
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
