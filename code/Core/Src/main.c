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
#include "stdio.h"
#include "servo.h"
#include "stepper.h"
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
UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
int MessageReceived =0;
char ReceivedMessage[100];
TypeServo J1Servo;
uint32_t PreviousSysTick=0;
uint8_t Homed=0;
uint8_t HomeLimitDetected=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==LPUART1){
		MessageReceived=1;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0){
	  HomeLimitDetected=1;
  }

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
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  char msg[1000];
  strcpy(msg,"Axis Starting ...\n");
  HAL_UART_Transmit_IT(&hlpuart1, msg,strlen(msg));
  HAL_Delay(1000);
  strcpy(msg,"Axis Ready!\n");
  HAL_UART_Transmit_IT(&hlpuart1, msg,strlen(msg));
  HAL_Delay(100);
  HAL_UART_Receive_DMA(&hlpuart1,&ReceivedMessage , 1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  init_servo(&J1Servo, "J1", 500, 2000, 10000, 10000, 10000, 180);
  J1Servo.target_position_degrees = 90.0f;

  set_motion_profile(&J1Servo, 2.0f, 2.0f, 70.0f);

  htim1.Instance->CCR1=500;//home servo to zero
  J1Servo.current_position_degrees=0;
  HAL_Delay(1000);



  uint32_t PreviousTimerValue=0;
  int PositionReached=0;

  Direction RequestedDirection=Stop;
  uint32_t RequestedSteps=1000;
  uint8_t MoveStepperCompleted=0;
  uint32_t StepTimeDelay=350;
  uint8_t HomingStep=10;
  int8_t msgSent=0;
  int8_t MotionSteps=10;
  int8_t ExecuteMotion=0;
  int32_t PreviousStepCount=0;
  uint32_t LastSysTick=0;
  while (1)
  {
	  if (HAL_GetTick()-PreviousSysTick>=10 && !PositionReached){
		  PositionReached= update_servo_state(&J1Servo, 0.01f, 1.0f);
		  htim1.Instance->CCR1=J1Servo.output_pwm;
		  PreviousSysTick= HAL_GetTick();
	  }

	  //Parse received commands
	  if (MessageReceived) {
		  if (ReceivedMessage[0]=='f') { // F for Forward
			  RequestedDirection=Forward;
			  RequestedSteps=20000;
			  StepTimeDelay=40;
			  sprintf(msg,"Moving Forward: %ld steps, %ld us intervals \n",RequestedSteps,StepTimeDelay);
			  MoveStepperCompleted=0;
		  }
		  else if (ReceivedMessage[0]=='r'){ // R for reverse
			  RequestedDirection=Reverse;
			  RequestedSteps=20000;
			  StepTimeDelay=40;
			  sprintf(msg,"Moving Reverse: %ld steps, %ld us intervals \n",RequestedSteps,StepTimeDelay);
			  MoveStepperCompleted=0;
		  }
		  else if (ReceivedMessage[0]=='s'){ // S for Stop
			  strcpy(msg,"Stop Command received \n");
			  StepCount=0;
			  RequestedDirection=Stop;
			  ExecuteMotion=0;
			  MotionSteps=10;
			  MoveStepperCompleted=1;
		  }
		  else if (ReceivedMessage[0]=='h'){
			  strcpy(msg,"Home Command received \n");
			  Homed=0; 		// h for Home
		  }
		  else if (ReceivedMessage[0]=='a'){ // Left
			  strcpy(msg,"Jog Forward \n");
			  RequestedDirection=Forward;
			  RequestedSteps=1;
			  StepTimeDelay=4000;
			  MoveStepperCompleted=0;
		  }
		  else if (ReceivedMessage[0]=='d'){ // Right
			  strcpy(msg,"Jog Reverse \n");
			  RequestedDirection=Reverse;
			  RequestedSteps=10;
			  StepTimeDelay=400;
			  MoveStepperCompleted=0;
		  }
		  else if (ReceivedMessage[0]=='1'){ // Right
			  strcpy(msg,"Executing Motion Profile 1 \n");
			  ExecuteMotion=1;
		  }
		  else {
			  sprintf(msg,"Undefined\n",ReceivedMessage[0]);
			  HAL_UART_Transmit_IT(&hlpuart1, msg,strlen(msg));
		  }
		  HAL_UART_Transmit_IT(&hlpuart1, msg,strlen(msg));
		  MessageReceived=0;
	  }
	  // Homing Sequence
	  if(!Homed){
		  switch (HomingStep){
		  case 10: //Move Forward a bit
			  if (!msgSent){
				  strcpy(msg,"Homing...\n");
				  HAL_UART_Transmit_IT(&hlpuart1,msg ,strlen(msg));
				  msgSent=1;
				  MoveStepperCompleted=0;
			  }
			  StepTimeDelay=300;
			  RequestedDirection=Forward;
			  RequestedSteps=500;
			  if (MoveStepperCompleted) {
				  HomingStep=20;
				  msgSent=0;
			  }
			  break;
		  case 20: // Move Backward until reaching home switch
			  if (!msgSent){
				  strcpy(msg,"Waiting for switch\n");
				  HAL_UART_Transmit_IT(&hlpuart1, msg ,strlen(msg));
				  msgSent=1;
				  MoveStepperCompleted=0;
			  }
			  RequestedDirection=Reverse;
			  RequestedSteps=10000000;
			  if (HomeLimitDetected) {
				  HomingStep=30;
				  msgSent=0;
			  }
			  break;
		  case 30: // Move to step 100 az new position 0
			  if (!msgSent){
				  strcpy(msg,"Moving to 0\n");
				  HAL_UART_Transmit_IT(&hlpuart1, msg ,strlen(msg));
				  msgSent=1;
				  MoveStepperCompleted=0;
			  }
			  RequestedDirection=Forward;
			  RequestedSteps=750;
			  if (MoveStepperCompleted) {
				  HomingStep=40;
				  msgSent=0;
			  }
			  break;
		  case 40:
			  if (!msgSent){
				  strcpy(msg,"Axis Homed!\n");
				  HAL_UART_Transmit_IT(&hlpuart1, msg ,strlen(msg));
				  msgSent=1;
			  }
			  Homed=1;
			  HomingStep=10;
			  break;
		  }
	  }

	  //Motion Sequence
	  if(ExecuteMotion){
		  uint32_t MinStepDelay=10;
		  uint32_t MaxStepDelay=1000;
		  uint32_t AccelSpan=500;

		  switch(MotionSteps){
		  case 10:
			  StepCount=0;
			  StepTimeDelay=MaxStepDelay;
			  RequestedDirection=Forward;
			  RequestedSteps=20000;
			  if(( HAL_GetTick()-LastSysTick)>=50 ||1) {
				  MoveStepperCompleted=0;
				  MotionSteps=20;
			  }
			  break;
		  case 20:
			  if (PreviousStepCount!=StepCount){
				  StepTimeDelay-=(MaxStepDelay-MinStepDelay)/AccelSpan;
				  PreviousStepCount=StepCount;
			  }
			  if (StepCount>=AccelSpan) MotionSteps=30;
			  break;
		  case 30:
			  //wait
			  if (StepCount>=(20000-AccelSpan)) MotionSteps=40;
			  break;
		  case 40:
			  if (PreviousStepCount!=StepCount){
				  StepTimeDelay+=(MaxStepDelay-MinStepDelay)/AccelSpan;
				  PreviousStepCount=StepCount;
			  }
			  LastSysTick= HAL_GetTick();
			  if (MoveStepperCompleted) MotionSteps=50;
			  break;
		  case 50:
			  StepCount=0;
			  RequestedDirection=Reverse;
			  RequestedSteps=20000;
			  if ( (HAL_GetTick()-LastSysTick)>=50||1) {
				  MoveStepperCompleted=0;
				  MotionSteps=60;
			  }
			  break;
		  case 60:
			  if (PreviousStepCount!=StepCount){
				  StepTimeDelay-=(MaxStepDelay-MinStepDelay)/AccelSpan;
				  PreviousStepCount=StepCount;
			  }
			  if (StepCount>=AccelSpan) MotionSteps=70;
			  break;
		  case 70:
			  //wait
			  if (StepCount>=(20000-AccelSpan)) MotionSteps=80;
			  break;
		  case 80:
			  if (PreviousStepCount!=StepCount){
				  StepTimeDelay+=(MaxStepDelay-MinStepDelay)/AccelSpan;
				  PreviousStepCount=StepCount;
			  }
			  LastSysTick= HAL_GetTick();
			  if (MoveStepperCompleted ) MotionSteps=10;
			  break;
		  }
	  }


	  // Move command happens here only
	  if (htim2.Instance->CNT -  PreviousTimerValue > StepTimeDelay ){ //MIN 350 AT 24vDC
		  if ( (RequestedDirection==Forward || RequestedDirection==Reverse) && !MoveStepperCompleted){
			  MoveStepperCompleted= MoveStepperTMC2209(RequestedDirection, RequestedSteps);
		  }
		  PreviousTimerValue=htim2.Instance->CNT;
	  }
	  //Limit switch on motor side kills and zeros everthing - this used for homing as well
	  if(HomeLimitDetected){
		  RequestedDirection=Stop;
		  StepCount=0;
		  HomeLimitDetected=0;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  htim1.Init.Prescaler = 170-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  htim2.Init.Prescaler = 170-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STEP_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_Pin DIR_Pin */
  GPIO_InitStruct.Pin = STEP_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : HomeSwitch_Pin */
  GPIO_InitStruct.Pin = HomeSwitch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(HomeSwitch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
