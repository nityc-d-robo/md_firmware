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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  INIT, STATUS, PWM, SPEED, ANGLE, LIM_SW
} Mode;
typedef struct {
	uint32_t cnt;
	int16_t overflow;
	int64_t fusion_cnt;		//cnt + overflow * 2
} Encoder;
typedef struct {
	bool port;
	bool state;
  int after_power;
} LimitSwitch;
typedef struct {
  int16_t target_count;
  int64_t p;
  int64_t pre_p;
  int64_t i;
  int64_t d;
} PidController ;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CPR 8192			//Count Per Rotation
#define SPEED_RATE 200  //control rate [Hz]
#define CAN_SIZE 8		//CAN send data size[Byte]
#define RETURN_SIZE 8	//CAN return data size[Byte]
#define ENCODER_COUNTERPERIOD 4294967295

#define Kp 1
#define Ki 1
#define Kd 1
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
Mode mode = INIT;
Encoder pre_encoder = {0};
Encoder encoder = {0};
PidController pid = {0};
LimitSwitch lim_sw = {0};

int16_t delta_count = 0;

uint32_t id_own = 0u;

// livewatch
int16_t power = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void stopAll(void);
void simplePWM(int16_t power_);
void rotateSpeed(int16_t target_count);
void returnStatus(uint8_t master_id_);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim_);
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin_);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_);
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
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  HAL_NVIC_DisableIRQ(EXTI1_IRQn);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  HAL_TIM_Base_Start_IT(&htim16);
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
  uint32_t id_sw;
  uint32_t id_all = 0xf0 << 21;
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
  id_sw += (uint32_t)(!HAL_GPIO_ReadPin(MODE3_GPIO_Port, MODE3_Pin)) << 3;
  id_sw += (uint32_t)(!HAL_GPIO_ReadPin(MODE2_GPIO_Port, MODE2_Pin)) << 2;
  id_sw += (uint32_t)(!HAL_GPIO_ReadPin(MODE1_GPIO_Port, MODE1_Pin)) << 1;
  id_sw += (uint32_t)(!HAL_GPIO_ReadPin(MODE0_GPIO_Port, MODE0_Pin));

  id_own = id_sw << 21;
  filter.FilterIdHigh = 0;
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = 0;
  filter.FilterMaskIdLow = 0;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.SlaveStartFilterBank = 14;
  filter.FilterActivation = ENABLE;
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
  htim2.Init.Period = 4294967295;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : MODE0_Pin MODE1_Pin MODE2_Pin MODE3_Pin
                           Z_Pin */
  GPIO_InitStruct.Pin = MODE0_Pin|MODE1_Pin|MODE2_Pin|MODE3_Pin
                          |Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW0_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW0_Pin|SW1_Pin;
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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void stopAll(void){
  mode = INIT;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}
void simplePWM(int16_t power_){
  bool phase = power_ > 0 ? false : true;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (GPIO_PinState)phase);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (abs(power_) < 1000) ? abs(power_) : 999);
}
void rotateSpeed(int16_t target_count) {  
  if(pid.i > 16384) {
    pid.i = 0;
    pid.target_count-=10;
  } else if(pid.i < -16384) {
    pid.i = 0;
    pid.target_count+=10;
  }
  pid.pre_p = pid.p;
  pid.p = Kp * (target_count - delta_count);
  pid.i += Ki * pid.p;
  pid.d = Kd * (pid.p - pid.pre_p) / 0.1;
  power = pid.p + pid.i + pid.d;
  simplePWM(power);
}

void returnStatus(uint8_t master_id_){
	CAN_TxHeaderTypeDef tx_header;
	uint32_t tx_mailbox;
	uint8_t tx_datas[RETURN_SIZE];

	int64_t angle = encoder.fusion_cnt % CPR;	//calculate encoder angle
  delta_count = encoder.fusion_cnt - pre_encoder.fusion_cnt;

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) <= 0);

	tx_header.StdId = master_id_;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.IDE = CAN_ID_STD;
	tx_header.DLC = RETURN_SIZE;
	tx_header.TransmitGlobalTime = DISABLE;
	tx_datas[0] = (uint8_t)((angle >> 8) & 0xff);
	tx_datas[1] = (uint8_t)(angle & 0xff);
	tx_datas[2] = (uint8_t)((delta_count >> 8) & 0xff);
	tx_datas[3] = (uint8_t)(delta_count & 0xff);
	tx_datas[4] = (uint8_t)(!HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin));
	tx_datas[5] = (uint8_t)(!HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin));
	tx_datas[6] = (uint8_t)((id_own >> 21) & 0xff);
	tx_datas[7] = 0;
	HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_datas, &tx_mailbox);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
      __HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_UPDATE);
      if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
			  encoder.overflow--;
		  }else{
			  encoder.overflow++;
		  }
    } else if (htim->Instance == TIM16) {
      __HAL_TIM_CLEAR_FLAG(&htim16, TIM_IT_UPDATE);
      switch(mode) {
        case INIT:
          HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
          break;
        case SPEED:
          rotateSpeed(pid.target_count);
          break;
      }
      pre_encoder = encoder;
      encoder.cnt = __HAL_TIM_GET_COUNTER(&htim2);
      encoder.fusion_cnt = encoder.cnt + encoder.overflow*ENCODER_COUNTERPERIOD;
      delta_count = encoder.fusion_cnt - pre_encoder.fusion_cnt;
      returnStatus(0x60);
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin_){
  if(mode != LIM_SW) return;
	if(gpio_pin_ == SW0_Pin && !lim_sw.port){
		simplePWM(lim_sw.after_power);
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	}else if(gpio_pin_ == SW1_Pin && lim_sw.port){
		simplePWM(lim_sw.after_power);
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	}
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_){
	CAN_RxHeaderTypeDef rx_header;
	uint32_t receive_id = 0u;
	uint8_t rx_data[CAN_SIZE] = { 0u };
	if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK){
		receive_id = (rx_header.IDE == CAN_ID_STD)? rx_header.StdId : rx_header.ExtId;
		mode == INIT;

		int master_id = rx_data[0];
		int semi_id = rx_data[1];

		if(receive_id == 0xf0){
			stopAll();
		} else if(receive_id == id_own >> 21){
			switch(rx_data[2]){
        case INIT:
          break;
        case STATUS:
          break;
				case PWM:
          mode = PWM;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					simplePWM((int16_t)(rx_data[4]<<8 | rx_data[5]));
					break;
				case SPEED:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
          int16_t target_count = (int16_t)(rx_data[4]<<8 | rx_data[5]);
          if(target_count == 0){
            simplePWM(0);
          }else {
            pid.target_count = target_count;
					  rotateSpeed(pid.target_count);
            mode = SPEED;
          }
					break;
				case LIM_SW:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
          lim_sw.port = (bool)rx_data[3];
          lim_sw.after_power = (int16_t)(rx_data[6]<<8 | rx_data[7]);
          if(lim_sw.port == 0) {
            if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin)) {
              break;  
            }
            HAL_NVIC_EnableIRQ(EXTI0_IRQn);
          } else {
            if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)){
              break;
            }
            HAL_NVIC_EnableIRQ(EXTI1_IRQn);
          }
          mode = LIM_SW;
          simplePWM((int16_t)(rx_data[4]<<8 | rx_data[5]));
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
