/* USER CODE BEGIN Header */
//Ver 3.0.0 2022/12/23 k-trash
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
	INIT, STATUS, PWM, SPEED, ANGLE, LIM_SW
}Mode;

typedef enum FinishStatus{
	F_STATUS = 0, F_SUCCESS, F_TIMEOUT, F_INTERRUPT, F_OTHER
}FinishStatus;

typedef struct RingBuf{
	uint8_t now_point;		//now point of ring buffer
	uint8_t buf_num;			//amount of ring buffer
	int32_t *buf;
}RingBuf;

typedef struct Encoder{
	uint16_t cnt;
	int16_t overflow;
	int64_t fusion_cnt;		//cnt + overflow * 65535
}Encoder;

typedef struct PID{
	float P_GAIN;
	float I_GAIN;
	float D_GAIN;

	RingBuf prop;
	int32_t diff;
	int32_t integ;
}PID;

typedef struct EncoderSpeed{
	bool phase;
	uint16_t rpm, end;		//Given first

	int32_t power;			//calculated from current rpm [pwm]
	int32_t end_power;		//calculated from end count [pwm]

	uint16_t count;
	uint16_t timeout;
	int32_t acc;			//accelaration
	uint32_t now_speed;		//average speed[slit]
	uint32_t target_speed;	//calculated from "rpm" [slit]
	int32_t delta;			//control amount
	uint32_t end_cnt;		//calculated from "end" [slit]

	Encoder first;			//first count status
	Encoder pre;			//at previous loop
	Encoder now;			//now count

	PID speed_pid;
	PID end_pid;

	Mode mode;
}EncoderSpeed;

typedef struct LimitSwitch{
	bool port;
	bool phase;
}LimitSwitch;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPR 250			//Slit Per Rotation
#define SPEED_RATE 200  //control rate [Hz]
#define CAN_SIZE 10		//CAN send data size[Byte]
#define RETURN_SIZE 8	//CAN return data size[Byte]
#define TAU 800			//Time constant of speed[ms]
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
void initDriver(bool angle_reset_ ,uint16_t max_rpm_, uint16_t max_torque_, uint16_t general_torque_);

void returnStatus(uint8_t master_id_, uint8_t semi_id_);

bool startSpeed(void);
bool initSpeed(bool phase_, uint16_t rpm_, uint16_t end_, uint16_t timeout_);
bool rotateSpeed(void);
void finishSpeed(FinishStatus finish_status_);

bool initAngle(uint16_t rpm_, int32_t angle_, uint16_t timeout_);

bool initLimit(bool phase_, uint16_t rpm_, bool port_, uint16_t timeout_);
void finishLimit(FinishStatus finish_status_);

void stopAll(void);
void simplePWM(bool phase_, uint16_t power_);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim_);
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin_);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile EncoderSpeed encoder;
volatile LimitSwitch limit_sw;

uint32_t id_own = 0u;							//CAN ID

volatile int32_t overflow = 0;
volatile bool speed_flag = false;
volatile bool limit_flag = false;
volatile bool angle_first = true;
volatile uint8_t angle_code = 0u;

volatile uint8_t master_id = 0x50;
volatile uint8_t semi_id = 0x40;

volatile uint16_t max_rpm = 20400u;				//maximum motor rpm[rpm]
volatile uint16_t max_torque = 1523u;			//maximum torque[g.cm]
volatile uint16_t general_torque = 0u;			//general torque[g.cm]
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
	uint32_t id_all = 0xf0 << 21;		//common ID
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
void initDriver(bool angle_reset_, uint16_t max_rpm_, uint16_t max_torque_, uint16_t general_torque_){
	if(max_rpm_ != 0u){
		max_rpm = max_rpm_;
	}

	if(max_torque_ != 0u){
		max_torque = max_torque_;
	}

	if(general_torque_ != 0u && max_torque > general_torque_){
		general_torque = general_torque_;
	}

	if(angle_reset_){
		overflow = 0;
		__HAL_TIM_SET_COUNTER(&htim2, 0u);
	}
}

void returnStatus(uint8_t master_id_, uint8_t semi_id_){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t tx_mailbox;
	uint8_t tx_datas[RETURN_SIZE];

	int32_t angle = __HAL_TIM_GET_COUNTER(&htim2) * 360 / (4*SPR) + overflow * 65535 / (4*SPR) * 360;	//calculate encoder angle

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) <= 0);

	TxHeader.StdId = semi_id_;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = RETURN_SIZE;
	TxHeader.TransmitGlobalTime = DISABLE;
	tx_datas[0] = id_own;
	tx_datas[1] = master_id_;
	tx_datas[2] = (uint8_t)F_STATUS;
	tx_datas[3] = 30u;														//Ver2.0
	tx_datas[4] = (uint8_t)((angle > 0) ? false : true);
	tx_datas[5] = (uint8_t)((abs(angle) >> 8) & 0xff);
	tx_datas[6] = (uint8_t)(abs(angle) & 0xff);
	tx_datas[7] = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) << 1 | !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx_datas, &tx_mailbox);
}

bool initSpeed(bool phase_, uint16_t rpm_, uint16_t end_, uint16_t timeout_){
	if(rpm_ == 0){
		return false;
	}

	encoder.phase = phase_;
	encoder.rpm = rpm_;
	encoder.end = end_;

	encoder.timeout = SPEED_RATE / 100 * timeout_ / 10;

	encoder.speed_pid.P_GAIN = 6;
	encoder.speed_pid.I_GAIN = 0.0001;
	encoder.speed_pid.D_GAIN = 0.01;

	encoder.end_pid.P_GAIN = 6;
	encoder.end_pid.I_GAIN = 0.0001;
	encoder.end_pid.D_GAIN = 0.01;

	encoder.mode = SPEED;

	startSpeed();

	return true;
}

bool startSpeed(void){
	encoder.speed_pid.prop.buf_num = 2u;
	encoder.speed_pid.prop.now_point = 0u;
	encoder.speed_pid.prop.buf = calloc(encoder.speed_pid.prop.buf_num, sizeof(int32_t));
	if(encoder.speed_pid.prop.buf == NULL){
		stopAll();
		return false;
	}

	encoder.end_pid.prop.buf_num = 2u;
	encoder.end_pid.prop.now_point = 0u;
	encoder.end_pid.prop.buf = calloc(encoder.end_pid.prop.buf_num, sizeof(int32_t));
	if(encoder.end_pid.prop.buf == NULL){
		stopAll();
		return false;
	}

	encoder.first.cnt = encoder.pre.cnt = __HAL_TIM_GET_COUNTER(&htim2);
	encoder.first.overflow = overflow;
	encoder.first.fusion_cnt = 0;

	encoder.pre.overflow = 0;
	encoder.pre.fusion_cnt = encoder.pre.cnt;

	encoder.now.overflow = 0;
	encoder.now.cnt = 0u;
	encoder.now.fusion_cnt = 0;

	encoder.power = 0u;
	encoder.target_speed = (uint32_t)((encoder.rpm*SPR*4)/(60*SPEED_RATE));
	encoder.delta = 0u;
	encoder.end_cnt = (uint32_t)((encoder.end*SPR*4)/360);

	encoder.speed_pid.diff = 0;
	encoder.speed_pid.integ = 0;

	encoder.end_pid.diff = 0;
	encoder.end_pid.integ = 0;

	encoder.acc = (int32_t)((1000*encoder.rpm/max_rpm) + (1000*general_torque/max_torque)) * 999 / SPEED_RATE / TAU;

	encoder.count = 0u;

	__HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim16);

	speed_flag = true;

	return true;
}

bool rotateSpeed(void){
	encoder.now.cnt = __HAL_TIM_GET_COUNTER(&htim2) - encoder.first.cnt;
	encoder.now.overflow = overflow - encoder.first.overflow;
	encoder.now.fusion_cnt = encoder.now.cnt + encoder.now.overflow * 65535;

	if(abs((int)encoder.now.fusion_cnt) >= (int)encoder.end_cnt-5 && encoder.end != 0){
		simplePWM(encoder.phase, 0);
		encoder.power = 0;
		finishSpeed(F_SUCCESS);
		return false;
	}else if(encoder.count > encoder.timeout && encoder.timeout != 0){
		simplePWM(encoder.phase, 0);
		encoder.power = 0;
		if(encoder.mode == LIM_SW){
			finishLimit(F_TIMEOUT);
		}else{
			finishSpeed(F_TIMEOUT);
		}
		return false;
	}

	encoder.now_speed = abs((int)(encoder.now.fusion_cnt - encoder.pre.fusion_cnt));

	encoder.speed_pid.prop.now_point = 1 - encoder.speed_pid.prop.now_point;
	encoder.speed_pid.prop.buf[encoder.speed_pid.prop.now_point] = encoder.target_speed - encoder.now_speed;
	encoder.speed_pid.integ += encoder.speed_pid.prop.buf[encoder.speed_pid.prop.now_point];
	encoder.speed_pid.diff = encoder.speed_pid.prop.buf[encoder.speed_pid.prop.now_point] - encoder.speed_pid.prop.buf[1-encoder.speed_pid.prop.now_point];

	encoder.delta = encoder.speed_pid.P_GAIN*encoder.speed_pid.prop.buf[encoder.speed_pid.prop.now_point] + encoder.speed_pid.I_GAIN*encoder.speed_pid.integ/SPEED_RATE + encoder.speed_pid.D_GAIN*encoder.speed_pid.diff*SPEED_RATE;
	encoder.power += encoder.delta;

	if(encoder.speed_pid.prop.buf[encoder.speed_pid.prop.now_point] == 0){
		general_torque = encoder.power * max_torque / 999 - (int32_t)encoder.rpm * max_torque / max_rpm;
	}

	if(encoder.end){
		encoder.end_pid.prop.now_point = 1 - encoder.end_pid.prop.now_point;
		encoder.end_pid.prop.buf[encoder.end_pid.prop.now_point] = encoder.end_cnt - abs((int)encoder.now.fusion_cnt);
		encoder.end_pid.integ += encoder.end_pid.prop.buf[encoder.speed_pid.prop.now_point];
		encoder.end_pid.diff = encoder.end_pid.prop.buf[encoder.end_pid.prop.now_point] - encoder.end_pid.prop.buf[1-encoder.end_pid.prop.now_point];
		encoder.end_power = encoder.end_pid.P_GAIN*encoder.end_pid.prop.buf[encoder.end_pid.prop.now_point] + encoder.end_pid.I_GAIN*encoder.end_pid.integ/SPEED_RATE + encoder.end_pid.D_GAIN*encoder.end_pid.diff*SPEED_RATE;

		if(encoder.power > encoder.end_power){
			encoder.power = encoder.end_power;
		}
	}

	if(encoder.now.fusion_cnt == encoder.first.cnt){
		encoder.power += encoder.acc;
	}

	simplePWM(encoder.phase, (uint16_t)encoder.power);

	encoder.pre.fusion_cnt = encoder.now.fusion_cnt;

	encoder.count++;

	return true;
}

void finishSpeed(FinishStatus finish_status_){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t tx_mailbox;
	uint8_t tx_datas[RETURN_SIZE];

	HAL_TIM_Base_Stop_IT(&htim16);
	free(encoder.speed_pid.prop.buf);
	free(encoder.end_pid.prop.buf);

	if(angle_first){
		bool code_tmp = (encoder.now.fusion_cnt >= 0) ? 0 : 1;

		angle_code = (encoder.phase == code_tmp) ? 0 : 1;

		angle_first = false;
	}

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) <= 0);

	TxHeader.StdId = semi_id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = RETURN_SIZE;
	TxHeader.TransmitGlobalTime = DISABLE;
	tx_datas[0] = id_own;
	tx_datas[1] = master_id;
	tx_datas[2] = (uint8_t)finish_status_;
	tx_datas[3] = (uint8_t)encoder.mode;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx_datas, &tx_mailbox);

	speed_flag = false;
}

bool initAngle(uint16_t rpm_, int32_t angle_, uint16_t timeout_){
	int32_t now_angle = ((int32_t)(__HAL_TIM_GET_COUNTER(&htim2) * 360 / (4*SPR) + overflow*65535 / (4*SPR) * 360));
	uint16_t target_angle = (uint16_t)abs(now_angle - angle_);

	if(rpm_ == 0){
		return false;
	}

	if(target_angle == 0){
		return false;
	}

	encoder.phase = (bool)((((now_angle - angle_ >= 0) ? 0 : 1) + angle_code) & 0x01);
	encoder.rpm = rpm_;
	encoder.end = target_angle;

	encoder.timeout = SPEED_RATE / 100 * timeout_ / 10;

	encoder.speed_pid.P_GAIN = 6;
	encoder.speed_pid.I_GAIN = 0.0001;
	encoder.speed_pid.D_GAIN = 0.01;

	encoder.end_pid.P_GAIN = 6;
	encoder.end_pid.I_GAIN = 0.0001;
	encoder.end_pid.D_GAIN = 0.01;

	encoder.mode = ANGLE;

	startSpeed();

	return true;
}

void stopAll(void){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	encoder.power = 0;
}

void simplePWM(bool phase_, uint16_t power_){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (GPIO_PinState)phase_);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (power_ < 1000) ? power_ : 999);
	encoder.power = (power_ < 1000) ? power_ : 999;
}

bool initLimit(bool phase_, uint16_t rpm_, bool port_, uint16_t timeout_){
	limit_sw.phase = phase_;
	limit_sw.port = port_;

	if(!port_){
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	}else{
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	}

	if(rpm_ == 0){
		return false;
	}

	encoder.phase = phase_;
	encoder.rpm = rpm_;
	encoder.end = 0u;

	encoder.timeout = SPEED_RATE / 100 * timeout_ / 10;

	encoder.speed_pid.P_GAIN = 6;
	encoder.speed_pid.I_GAIN = 0.0001;
	encoder.speed_pid.D_GAIN = 0.01;

	encoder.end_pid.P_GAIN = 6;
	encoder.end_pid.I_GAIN = 0.0001;
	encoder.end_pid.D_GAIN = 0.01;

	encoder.mode = LIM_SW;

	limit_flag = true;

	startSpeed();

	return true;
}

void finishLimit(FinishStatus finish_status_){
	if(!limit_sw.port){
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	}else{
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	}

	limit_flag = false;

	finishSpeed(finish_status_);
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
	if(gpio_pin_ == GPIO_PIN_0 && !limit_sw.port){
		simplePWM(encoder.phase, 0u);
		finishLimit(F_SUCCESS);
	}else if(gpio_pin_ == GPIO_PIN_1 && limit_sw.port){
		simplePWM(encoder.phase, 0u);
		finishLimit(F_SUCCESS);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_){
	CAN_RxHeaderTypeDef RxHeader;
	uint32_t receive_id = 0u;
	uint8_t rx_data[CAN_SIZE] = { 0u };
	if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK){
		receive_id = (RxHeader.IDE == CAN_ID_STD)? RxHeader.StdId : RxHeader.ExtId;
		if(limit_flag){
			finishLimit(F_INTERRUPT);
		}else if(speed_flag){
			finishSpeed(F_INTERRUPT);
		}

		master_id = rx_data[0];
		semi_id = rx_data[1];

		if(receive_id == 0xf0){
			stopAll();
		}else{
			switch(rx_data[2]){
				case INIT:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					initDriver((bool)rx_data[3], (uint16_t)(rx_data[4]<<8 | rx_data[5]), (uint16_t)(rx_data[6]<<8 | rx_data[7]), (uint16_t)(rx_data[8]<<8 | rx_data[9]));
					break;
				case STATUS:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					returnStatus(rx_data[0], rx_data[1]);
					break;
				case PWM:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					simplePWM((bool)rx_data[3], (uint16_t)(rx_data[4]<<8 | rx_data[5]));
					break;
				case SPEED:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					initSpeed((bool)rx_data[3], (uint16_t)(rx_data[4]<<8 | rx_data[5]), (uint16_t)(rx_data[6]<<8 | rx_data[7]), (uint16_t)(rx_data[8]<<8 | rx_data[9]));
					break;
				case ANGLE:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					initAngle((uint16_t)(rx_data[4]<<8 | rx_data[5]), (int32_t)(rx_data[6]<<8 | rx_data[7])*(((rx_data[3]&0x01)==0) ? 1 : -1), (uint16_t)(rx_data[8]<<8 | rx_data[9]));
					break;
				case LIM_SW:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					initLimit((bool)rx_data[3], (uint16_t)(rx_data[4]<<8 | rx_data[5]), (bool)(rx_data[6]), (uint16_t)(rx_data[8]<<8 | rx_data[9]));
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
