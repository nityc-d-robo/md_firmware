/* USER CODE BEGIN Header */
//Ver 2.0.0 2022/01/19 k-trash
//writing...
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum Mode{
	INIT, PWM, SPEED, ANGLE, LIM_SW, STATUS
}Mode;

typedef struct Encoder{
	uint16_t cnt;
	int16_t overflow;
	uint64_t fusion_cnt;		//cnt + overflow * 65535
}Encoder;

typedef struct RingBuf{
	uint8_t now_point;		//now point of ring buffer
	uint8_t buf_num;			//amount of ring buffer
	int32_t *buf;
}RingBuf;

typedef struct PID{
	uint32_t P_GAIN;
	uint32_t I_GAIN;
	uint32_t D_GAIN;

	RingBuf propotion;
	int32_t diff;
	int32_t integration;
}PID;

typedef struct EncoderSpeed{
	bool phase;
	uint16_t rpm, end;		//Given first

	int32_t power;			//calculated from current rpm [pwm]
	int32_t end_power;		//calculated from end count [pwm]

	uint32_t average_speed;	//average speed[slit]
	uint32_t target_speed;	//calculated from "rpm" [slit]
	int32_t delta;			//control amount
	uint32_t end_cnt;			//calculated from "end" [slit]

	RingBuf speeds;			//ring buffer for save past speeds [slit]

	Encoder first;			//first count status
	Encoder pre;				//at previous loop
	Encoder now;				//now count

	PID speed_pid;
	PID end_pid;
}EncoderSpeed;

typedef struct LimitSwitch{
	bool port;
	bool phase;
	uint16_t power;
}LimitSwitch;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPR 48			//Slit Per Rotation
#define SPEED_RATE 200  //control rate [Hz]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void initDriver(uint16_t motor_max_rpm_, uint8_t gear_rate_, bool angle_reset_);

bool initSpeed(bool phase_, uint16_t rpm_, uint16_t end_);
bool rotateSpeed(void);
void finishSpeed(void);

void initLimit(bool phase_, uint16_t power_, bool port_);
void finishLimit(void);

void stopAll(void);
void simplePWM(bool phase_, uint16_t power_);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim_);
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin_);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile EncoderSpeed encoder_speed;
volatile LimitSwitch limit_switch;

volatile int32_t overflow = 0;
volatile bool speed_flag = false;
volatile bool limit_flag = false;

volatile uint16_t motor_max_rpm = 20400u;
volatile uint16_t gear_rate = 1u;
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
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  HAL_NVIC_DisableIRQ(EXTI1_IRQn);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  overflow = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(500);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef filter;
	uint32_t id_sw = 0u;
	uint32_t id_all = 0x100 << 21;		//共通ID(非常停止用)
	uint32_t id_own = 0u;				//基板のID
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  id_sw += (uint32_t)(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)) << 3;
  id_sw += (uint32_t)(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)) << 2;
  id_sw += (uint32_t)(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) << 1;
  id_sw += (uint32_t)(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));

  id_own = id_sw << 21;
  filter.FilterIdHigh         = id_all >> 16;
  filter.FilterIdLow          = id_all;
  filter.FilterMaskIdHigh     = id_own >> 16;
  filter.FilterMaskIdLow      = id_own;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterBank           = 0;
  filter.FilterMode           = CAN_FILTERMODE_IDLIST;
  filter.SlaveStartFilterBank = 14;
  filter.FilterActivation     = ENABLE;

  HAL_CAN_ConfigFilter(&hcan, &filter);
  /* USER CODE END CAN_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 239;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void initDriver(uint16_t motor_max_rpm_, uint8_t gear_rate_, bool angle_reset_){
	if(motor_max_rpm_ != 0u){
		motor_max_rpm = motor_max_rpm;
	}
	if(gear_rate_ != 0u){
		gear_rate = gear_rate_;
	}
	if(angle_reset_){
		overflow = 0;
		__HAL_TIM_SET_COUNTER(&htim2, 0u);
	}
}

bool initSpeed(bool phase_, uint16_t rpm_, uint16_t end_){
	if(rpm_ == 0){
		return false;
	}

	encoder_speed.speeds.buf_num = 4u;
	encoder_speed.speeds.now_point = 0u;
	encoder_speed.speeds.buf = calloc(encoder_speed.speeds.buf_num, sizeof(int32_t));
	if(encoder_speed.speeds.buf == NULL){
		stopAll();
		return false;
	}

	encoder_speed.speed_pid.propotion.buf_num = 2u;
	encoder_speed.speed_pid.propotion.now_point = 0u;
	encoder_speed.speed_pid.propotion.buf = calloc(encoder_speed.speed_pid.propotion.buf_num, sizeof(int32_t));
	if(encoder_speed.speed_pid.propotion.buf == NULL){
		stopAll();
		return false;
	}

	encoder_speed.end_pid.propotion.buf_num = 2u;
	encoder_speed.end_pid.propotion.now_point = 0u;
	encoder_speed.end_pid.propotion.buf = calloc(encoder_speed.end_pid.propotion.buf_num, sizeof(int32_t));
	if(encoder_speed.end_pid.propotion.buf == NULL){
		stopAll();
		return false;
	}

	encoder_speed.phase = phase_;
	encoder_speed.rpm = rpm_;
	encoder_speed.end = end_;

	encoder_speed.first.overflow = 0;
	encoder_speed.first.cnt = 0u;
	encoder_speed.first.fusion_cnt = 0;

	encoder_speed.pre.overflow = 0;
	encoder_speed.pre.cnt = 0u;
	encoder_speed.pre.fusion_cnt = 0;

	encoder_speed.now.overflow = 0;
	encoder_speed.now.cnt = 0u;
	encoder_speed.now.fusion_cnt = 0;

	encoder_speed.first.cnt = encoder_speed.pre.cnt = __HAL_TIM_GET_COUNTER(&htim2);
	encoder_speed.first.overflow = encoder_speed.pre.overflow = overflow;

	encoder_speed.power = 0u;
	encoder_speed.target_speed = (uint32_t)((rpm_*SPR*4)/(60*SPEED_RATE));
	encoder_speed.delta = 0u;
	encoder_speed.end_cnt = (uint32_t)((end_*SPR*4)/360);

	encoder_speed.speed_pid.P_GAIN = 20;
	encoder_speed.speed_pid.I_GAIN = 1;
	encoder_speed.speed_pid.D_GAIN = 10;
	encoder_speed.speed_pid.diff = 0;
	encoder_speed.speed_pid.integration = 0;

	encoder_speed.end_pid.P_GAIN = 100;
	encoder_speed.end_pid.I_GAIN = 1;
	encoder_speed.end_pid.D_GAIN = 10;
	encoder_speed.end_pid.diff = 0;
	encoder_speed.end_pid.integration = 0;

	__HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim16);

	speed_flag = true;

	return true;
}

bool rotateSpeed(void){
	encoder_speed.now.cnt = __HAL_TIM_GET_COUNTER(&htim2);
	encoder_speed.now.overflow = overflow - encoder_speed.first.overflow;
	encoder_speed.now.fusion_cnt = encoder_speed.now.cnt + encoder_speed.now.overflow * 65535;

	if(abs((int32_t)(encoder_speed.now.fusion_cnt - encoder_speed.first.cnt)) >= encoder_speed.end_cnt-5){				//change PID someday
		finishSpeed();
		return false;
	}

	encoder_speed.speeds.buf[(encoder_speed.speeds.now_point)++] = abs((int32_t)(encoder_speed.now.fusion_cnt - encoder_speed.pre.fusion_cnt));
	if(encoder_speed.speeds.now_point >= encoder_speed.speeds.buf_num){
		encoder_speed.speeds.now_point = 0;
	}
	encoder_speed.average_speed = (encoder_speed.speeds.buf[0] + encoder_speed.speeds.buf[1] + encoder_speed.speeds.buf[2] + encoder_speed.speeds.buf[3]) / encoder_speed.speeds.buf_num;

	encoder_speed.speed_pid.propotion.now_point = 1 - encoder_speed.speed_pid.propotion.now_point;
	encoder_speed.speed_pid.propotion.buf[encoder_speed.speed_pid.propotion.now_point] = encoder_speed.target_speed - encoder_speed.average_speed;
	encoder_speed.speed_pid.integration += encoder_speed.speed_pid.propotion.buf[encoder_speed.speed_pid.propotion.now_point];
	encoder_speed.speed_pid.diff = encoder_speed.speed_pid.propotion.buf[encoder_speed.speed_pid.propotion.now_point] - encoder_speed.speed_pid.propotion.buf[1-encoder_speed.speed_pid.propotion.now_point];

	encoder_speed.delta = encoder_speed.speed_pid.P_GAIN*encoder_speed.speed_pid.propotion.buf[encoder_speed.speed_pid.propotion.now_point] + encoder_speed.speed_pid.I_GAIN*encoder_speed.speed_pid.integration/SPEED_RATE + encoder_speed.speed_pid.D_GAIN*encoder_speed.speed_pid.diff*SPEED_RATE;
	encoder_speed.power += encoder_speed.delta;

	if(encoder_speed.end){
		encoder_speed.end_pid.propotion.now_point = 1 - encoder_speed.end_pid.propotion.now_point;
		encoder_speed.end_pid.propotion.buf[encoder_speed.end_pid.propotion.now_point] = encoder_speed.end_cnt - abs((int32_t)(encoder_speed.now.fusion_cnt - encoder_speed.first.cnt));
		encoder_speed.end_pid.integration += encoder_speed.end_pid.propotion.buf[encoder_speed.speed_pid.propotion.now_point];
		encoder_speed.end_pid.diff = encoder_speed.end_pid.propotion.buf[encoder_speed.end_pid.propotion.now_point] - encoder_speed.end_pid.propotion.buf[1-encoder_speed.end_pid.propotion.now_point];
		encoder_speed.end_power = encoder_speed.end_pid.P_GAIN*encoder_speed.end_pid.propotion.buf[encoder_speed.end_pid.propotion.now_point] + encoder_speed.end_pid.I_GAIN*encoder_speed.end_pid.integration + encoder_speed.end_pid.D_GAIN*encoder_speed.end_pid.diff;

		if(encoder_speed.power > encoder_speed.end_power){
			encoder_speed.power = encoder_speed.end_power;
		}
	}

	if(encoder_speed.now.fusion_cnt == encoder_speed.first.cnt){
		encoder_speed.power = (motor_max_rpm*SPR*4) / (60*SPEED_RATE*gear_rate) * 999 / encoder_speed.target_speed;
	}
	if(encoder_speed.power > 999){
		encoder_speed.power = 999;
	}else if(encoder_speed.power < 100){
		encoder_speed.power = 100;
	}

	simplePWM(encoder_speed.phase, encoder_speed.power);
	encoder_speed.pre.cnt = encoder_speed.now.cnt;
	encoder_speed.pre.overflow = encoder_speed.now.overflow;
	encoder_speed.pre.fusion_cnt = encoder_speed.now.fusion_cnt;

	return true;
}

void finishSpeed(void){
	simplePWM(encoder_speed.phase, 0);
	HAL_TIM_Base_Stop_IT(&htim16);
	free(encoder_speed.speeds.buf);
	free(encoder_speed.speed_pid.propotion.buf);
	free(encoder_speed.end_pid.propotion.buf);
	speed_flag = false;
}

void stopAll(void){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void simplePWM(bool phase_, uint16_t power_){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (GPIO_PinState)phase_);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, power_);
}

void initLimit(bool phase_, uint16_t power_, bool port_){
	limit_switch.phase = phase_;
	limit_switch.power = power_;
	limit_switch.port = port_;

	if(!port_){
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	}else{
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	}

	limit_flag = true;

	simplePWM(phase_, power_);

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void finishLimit(void){
	if(!limit_switch.port){
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	}else{
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	}

	limit_flag = false;

	simplePWM(limit_switch.phase, 0u);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim_){
	if(htim_->Instance == TIM2){
		__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_UPDATE);
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
			overflow--;
		}else{
			overflow++;
		}
	}else if(htim_->Instance == TIM16){
		__HAL_TIM_CLEAR_FLAG(&htim16, TIM_IT_UPDATE);
		rotateSpeed();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin_){
	if(gpio_pin_ == GPIO_PIN_0 && !limit_switch.port){
		finishLimit();
	}
	if(gpio_pin_ == GPIO_PIN_1 && limit_switch.port){
		finishLimit();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_){
	CAN_RxHeaderTypeDef RxHeader;
	uint32_t receive_id = 0u;
	uint8_t rx_data[6] = { 0u };
	if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK){
		receive_id = (RxHeader.IDE == CAN_ID_STD)? RxHeader.StdId : RxHeader.ExtId;
		if(speed_flag){
			finishSpeed();
		}else if(limit_flag){
			finishLimit();
		}
		if(receive_id == 0x100){
			stopAll();
		}else{
			switch(rx_data[0]){
				case INIT:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					initDriver((uint16_t)(rx_data[1]<<8 | rx_data[2]), rx_data[3], (bool)rx_data[4]);
					break;
				case PWM:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					simplePWM((bool)rx_data[1], (uint16_t)(rx_data[2]<<8 | rx_data[3]));
					break;
				case SPEED:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					initSpeed((bool)rx_data[1], (uint16_t)(rx_data[2]<<8 | rx_data[3]), (uint16_t)(rx_data[4]<<8 | rx_data[5]));
					break;
				case LIM_SW:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					initLimit((bool)rx_data[1], (uint16_t)(rx_data[2]<<8 | rx_data[3]), (bool)(rx_data[4]));
					break;
				default:
					stopAll();
					break;
			}
		}
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
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_Delay(500);
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
