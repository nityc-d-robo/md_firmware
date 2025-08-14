/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fdcan.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint32_t cnt;
  int16_t overflow;
  int64_t fusion_cnt;
} Encoder;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPEED_RATE 200  //control rate [Hz]
#define CAN_SIZE 8		//CAN send data size[Byte]
#define RETURN_SIZE 8	//CAN return data size[Byte]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Encoder encoder = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  __HAL_GPIO_EXTI_CLEAR_IT(FORWORD_SW_Pin);
  __HAL_GPIO_EXTI_CLEAR_IT(REVERSE_SW_Pin);

  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_IT_UPDATE);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

  HAL_TIM_Base_Start_IT(&htim17);

  HAL_GPIO_WritePin(GPIOA, STATE_LED_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // FDCAN_TxHeaderTypeDef   TxHeader;
    // uint8_t               TxData[] = {'H', 'E', 'L', 'L', 'O'};

    // // 送信ヘッダーの設定
    // TxHeader.Identifier = 0x0001;                      // 送信ID (11-bit Standard ID)
    // TxHeader.IdType = FDCAN_STANDARD_ID;              // IDタイプ
    // TxHeader.TxFrameType = FDCAN_DATA_FRAME;          // データフレーム
    // TxHeader.DataLength = FDCAN_DLC_BYTES_5;          // データ長 (5バイト)
    // TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    // TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           // CAN-FDのビットレート切り替え (OFF)
    // TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 従来のCANフォーマット
    // TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 送信イベントは記録しない
    // TxHeader.MessageMarker = 0;
    // HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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

/* USER CODE BEGIN 4 */
void setPWM(int16_t power_) {
  bool phase = power_ > 0 ? false : true;
  HAL_GPIO_WritePin(GPIOB, PHASE_Pin, (GPIO_PinState)phase);
  int pwm_duty = (abs(power_) < 1000) ? abs(power_) : 999;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_duty);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_IT_UPDATE);
    encoder.cnt = __HAL_TIM_GET_COUNTER(&htim1);
    if (encoder.cnt < 32768) {
      encoder.overflow++;
    } else {
      encoder.overflow--;
    }
  } else if (htim->Instance == TIM17) {
    encoder.cnt = __HAL_TIM_GET_COUNTER(&htim1);
    encoder.fusion_cnt = encoder.cnt + encoder.overflow * 65536LL;
    setPWM(500);
  }
  
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin_){
  switch (gpio_pin_) {
    case LIMIT_SW1_Pin:
      break;
    case LIMIT_SW2_Pin:
      break;
    case FORWORD_SW_Pin:
      HAL_GPIO_TogglePin(GPIOF, FORWORD_LED_Pin);
      break;
    case REVERSE_SW_Pin:
      HAL_GPIO_TogglePin(GPIOA, REVERSE_LED_Pin);
      break;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan_, uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef rx_header;
  uint32_t receive_id = 0u;
	uint8_t rx_data[CAN_SIZE] = { 0u };

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == RESET) { return; }
  if (HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
    receive_id = rx_header.Identifier;
    HAL_GPIO_TogglePin(GPIOF, FORWORD_LED_Pin);
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
