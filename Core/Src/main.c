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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cli.h"
#include "stm32f2xx_hal_gpio.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static cli_status_t set_func(int argc, char **argv);
static cli_status_t reset_func(int argc, char **argv);
static cli_status_t read_func(int argc, char **argv);
static cli_status_t help_func(int argc, char **argv);
static cli_status_t vsense_func(int argc, char **argv);
void user_uart_println(char *string);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
cmd_t cmd_tbl[] = {
    {.cmd = "set", .func = set_func},
    {.cmd = "reset", .func = reset_func},
    {.cmd = "read", .func = read_func},
    {.cmd = "help", .func = help_func},
    {.cmd = "vsense", .func = vsense_func},
};

cli_t cli;
uint8_t rx_data[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  // configuring the cli lib
  cli.println = user_uart_println;
  cli.cmd_tbl = cmd_tbl;
  cli.cmd_cnt = sizeof(cmd_tbl) / sizeof(cmd_t);
  HAL_GPIO_WritePin(LATCH_RST_GPIO_Port, LATCH_RST_Pin, GPIO_PIN_SET);
  cli.println("VCU (v1.0) Test Suite v0.1\r\n");
  cli_init(&cli);
  // configuring the HAL uart interupt
  HAL_UART_Receive_IT(&huart1, rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    cli_process(&cli);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
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
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void) {

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC,
                    HVCN_EN_Pin | HVCP_EN_Pin | CHRGP_EN_Pin | CHRGN_EN_Pin |
                        PRECHRG_EN_Pin | PUMP_EN_Pin | LATCH_RST_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DEBUG_1_Pin | DEBUG_2_Pin | IMD_IO_H_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_OK_GPIO_Port, MCU_OK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HVCN_EN_Pin HVCP_EN_Pin CHRGP_EN_Pin CHRGN_EN_Pin
                           PRECHRG_EN_Pin PUMP_EN_Pin LATCH_RST_Pin */
  GPIO_InitStruct.Pin = HVCN_EN_Pin | HVCP_EN_Pin | CHRGP_EN_Pin |
                        CHRGN_EN_Pin | PRECHRG_EN_Pin | PUMP_EN_Pin |
                        LATCH_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_OUT2_Pin BMS_OUT3_Pin */
  GPIO_InitStruct.Pin = BMS_OUT2_Pin | BMS_OUT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SAFETY_LOOP_ST_Pin */
  GPIO_InitStruct.Pin = SAFETY_LOOP_ST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SAFETY_LOOP_ST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IGNITION_SW_Pin */
  GPIO_InitStruct.Pin = IGNITION_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IGNITION_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CHARGE_SW_Pin HVCP_AUX_Pin HVCN_AUX_Pin
   * PRECHRG_AUX_Pin */
  GPIO_InitStruct.Pin =
      CHARGE_SW_Pin | HVCP_AUX_Pin | HVCN_AUX_Pin | PRECHRG_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_1_Pin DEBUG_2_Pin IMD_IO_H_Pin */
  GPIO_InitStruct.Pin = DEBUG_1_Pin | DEBUG_2_Pin | IMD_IO_H_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_OK_Pin */
  GPIO_InitStruct.Pin = MCU_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCU_OK_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_UART_Receive_IT(&huart1, rx_data, 1);
  cli_put(&cli, rx_data[0]);
  user_uart_println((char *)rx_data);
}

void user_uart_println(char *string) {
  HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen(string), HAL_MAX_DELAY);
}

cli_status_t set_func(int argc, char **argv) {
  if (argc == 2) {
    if (strcmp(argv[1], "--help") == 0) {
      cli.println("Pass the pin name as an argument to the function...");
    } else if (strcmp(argv[1], "MCU_OK") == 0) {
      HAL_GPIO_WritePin(MCU_OK_GPIO_Port, MCU_OK_Pin, GPIO_PIN_SET);
      cli.println("SET MCU_OK");
    } else if (strcmp(argv[1], "DEBUG_1") == 0) {
      HAL_GPIO_WritePin(DEBUG_1_GPIO_Port, DEBUG_1_Pin, GPIO_PIN_SET);
      cli.println("SET DEBUG_1");
    } else if (strcmp(argv[1], "DEBUG_2") == 0) {
      HAL_GPIO_WritePin(DEBUG_2_GPIO_Port, DEBUG_2_Pin, GPIO_PIN_SET);
      cli.println("SET DEBUG_2");
    } else if (strcmp(argv[1], "HVCN_EN") == 0) {
      HAL_GPIO_WritePin(HVCN_EN_GPIO_Port, HVCN_EN_Pin, GPIO_PIN_SET);
      cli.println("SET HVCN_EN");
    } else if (strcmp(argv[1], "HVCP_EN") == 0) {
      HAL_GPIO_WritePin(HVCP_EN_GPIO_Port, HVCP_EN_Pin, GPIO_PIN_SET);
      cli.println("SET HVCP_EN");
    } else if (strcmp(argv[1], "CHRGN_EN") == 0) {
      HAL_GPIO_WritePin(CHRGN_EN_GPIO_Port, CHRGN_EN_Pin, GPIO_PIN_SET);
      cli.println("SET CHRGN_EN");
    } else if (strcmp(argv[1], "CHRGP_EN") == 0) {
      HAL_GPIO_WritePin(CHRGP_EN_GPIO_Port, CHRGP_EN_Pin, GPIO_PIN_SET);
      cli.println("SET CHRGP_EN");
    } else if (strcmp(argv[1], "PRECHRG_EN") == 0) {
      HAL_GPIO_WritePin(PRECHRG_EN_GPIO_Port, PRECHRG_EN_Pin, GPIO_PIN_SET);
      cli.println("SET PRECHRG_EN");
    } else if (strcmp(argv[1], "PUMP_EN") == 0) {
      HAL_GPIO_WritePin(PUMP_EN_GPIO_Port, PUMP_EN_Pin, GPIO_PIN_SET);
      cli.println("SET PUMP_EN");
    } else if (strcmp(argv[1], "LATCH_RST") == 0) {
      HAL_GPIO_WritePin(LATCH_RST_GPIO_Port, LATCH_RST_Pin, GPIO_PIN_SET);
      cli.println("SET LATCH_RST");
    } else {
      cli.println("ERROR: Invalid Pin Name\r\nPass the pin name as an argument "
                  "to the function...");
      return CLI_E_INVALID_ARGS;
    }
  } else {
    cli.println("ERROR: `set` Requires One Pin Name\r\nPass the pin name as an "
                "argument to the function...");
    return CLI_E_INVALID_ARGS;
  }
  return CLI_OK;
}

cli_status_t reset_func(int argc, char **argv) {
  if (argc == 2) {
    if (strcmp(argv[1], "--help") == 0) {
      cli.println("Pass the pin name as an argument to the function...");
    } else if (strcmp(argv[1], "MCU_OK") == 0) {
      HAL_GPIO_WritePin(MCU_OK_GPIO_Port, MCU_OK_Pin, GPIO_PIN_RESET);
      cli.println("RESET MCU_OK");
    } else if (strcmp(argv[1], "DEBUG_1") == 0) {
      HAL_GPIO_WritePin(DEBUG_1_GPIO_Port, DEBUG_1_Pin, GPIO_PIN_RESET);
      cli.println("RESET DEBUG_1");
    } else if (strcmp(argv[1], "DEBUG_2") == 0) {
      HAL_GPIO_WritePin(DEBUG_2_GPIO_Port, DEBUG_2_Pin, GPIO_PIN_RESET);
      cli.println("RESET DEBUG_2");
    } else if (strcmp(argv[1], "HVCN_EN") == 0) {
      HAL_GPIO_WritePin(HVCN_EN_GPIO_Port, HVCN_EN_Pin, GPIO_PIN_RESET);
      cli.println("RESET HVCN_EN");
    } else if (strcmp(argv[1], "HVCP_EN") == 0) {
      HAL_GPIO_WritePin(HVCP_EN_GPIO_Port, HVCP_EN_Pin, GPIO_PIN_RESET);
      cli.println("RESET HVCP_EN");
    } else if (strcmp(argv[1], "CHRGN_EN") == 0) {
      HAL_GPIO_WritePin(CHRGN_EN_GPIO_Port, CHRGN_EN_Pin, GPIO_PIN_RESET);
      cli.println("RESET CHRGN_EN");
    } else if (strcmp(argv[1], "CHRGP_EN") == 0) {
      HAL_GPIO_WritePin(CHRGP_EN_GPIO_Port, CHRGP_EN_Pin, GPIO_PIN_RESET);
      cli.println("RESET CHRGP_EN");
    } else if (strcmp(argv[1], "PRECHRG_EN") == 0) {
      HAL_GPIO_WritePin(PRECHRG_EN_GPIO_Port, PRECHRG_EN_Pin, GPIO_PIN_RESET);
      cli.println("RESET PRECHRG_EN");
    } else if (strcmp(argv[1], "PUMP_EN") == 0) {
      HAL_GPIO_WritePin(PUMP_EN_GPIO_Port, PUMP_EN_Pin, GPIO_PIN_RESET);
      cli.println("RESET PUMP_EN");
    } else if (strcmp(argv[1], "LATCH_RST") == 0) {
      HAL_GPIO_WritePin(LATCH_RST_GPIO_Port, LATCH_RST_Pin, GPIO_PIN_RESET);
      cli.println("RESET LATCH_RST");
    } else {
      cli.println("ERROR: Invalid Pin Name\r\nPass the pin name as an argument "
                  "to the function...");
      return CLI_E_INVALID_ARGS;
    }
  } else {
    cli.println(
        "ERROR: `reset` Requires One Pin Name\r\nPass the pin name as an "
        "argument to the function...");
    return CLI_E_INVALID_ARGS;
  }
  return CLI_OK;
}

cli_status_t read_func(int argc, char **argv) {
  GPIO_PinState pin_state;
  if (argc == 2) {
    if (strcmp(argv[1], "--help") == 0) {
      cli.println("Pass the pin name as an argument to the function...");
    } else if (strcmp(argv[1], "SAFETY_LOOP_ST") == 0) {
      pin_state =
          HAL_GPIO_ReadPin(SAFETY_LOOP_ST_GPIO_Port, SAFETY_LOOP_ST_Pin);
    } else if (strcmp(argv[1], "BMS_OUT2") == 0) {
      pin_state = HAL_GPIO_ReadPin(BMS_OUT2_GPIO_Port, BMS_OUT2_Pin);
    } else if (strcmp(argv[1], "BMS_OUT3") == 0) {
      pin_state = HAL_GPIO_ReadPin(BMS_OUT3_GPIO_Port, BMS_OUT3_Pin);
    } else if (strcmp(argv[1], "CHARGE_SW") == 0) {
      pin_state = HAL_GPIO_ReadPin(CHARGE_SW_GPIO_Port, CHARGE_SW_Pin);
    } else if (strcmp(argv[1], "IGNITION_SW") == 0) {
      pin_state = HAL_GPIO_ReadPin(IGNITION_SW_GPIO_Port, IGNITION_SW_Pin);
    } else if (strcmp(argv[1], "HVCP_AUX") == 0) {
      pin_state = HAL_GPIO_ReadPin(HVCP_AUX_GPIO_Port, HVCP_AUX_Pin);
    } else if (strcmp(argv[1], "HVCN_AUX") == 0) {
      pin_state = HAL_GPIO_ReadPin(HVCN_AUX_GPIO_Port, HVCN_AUX_Pin);
    } else if (strcmp(argv[1], "PRECHRG_AUX") == 0) {
      pin_state = HAL_GPIO_ReadPin(PRECHRG_AUX_GPIO_Port, PRECHRG_AUX_Pin);
    } else {
      cli.println("ERROR: Invalid Pin Name\r\nPass the pin name as an argument "
                  "to the function...");
      return CLI_E_INVALID_ARGS;
    }
  } else {
    cli.println(
        "ERROR: `read` Requires One Pin Name\r\nPass the pin name as an "
        "argument to the function...");
    return CLI_E_INVALID_ARGS;
  }
  if (pin_state == GPIO_PIN_SET) {
    cli.println("pin is HIGH");
  } else if (pin_state == GPIO_PIN_RESET) {
    cli.println("pin is LOW");
  } else {
    cli.println("pin is UNKNOWN");
  }
  return CLI_OK;
}
cli_status_t help_func(int argc, char **argv) {
  cli.println("deal with this later...");
  return CLI_OK;
}
cli_status_t vsense_func(int argc, char **argv) {
  HAL_ADC_Start(&hadc1); // Needs to be called every time
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint16_t value_adc_high = HAL_ADC_GetValue(&hadc1);
  char buf[50];
  sprintf(buf, "adc_high value %i\r\n", value_adc_high);
  cli.println(buf);
  HAL_ADC_Start(&hadc2); // Needs to be called every time
  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
  uint16_t value_adc_low = HAL_ADC_GetValue(&hadc2);
  sprintf(buf, "adc_high value %i\r\n", value_adc_low);
  cli.println(buf);
  uint16_t vsense_unscaled = value_adc_high - value_adc_low;
  sprintf(buf, "adc value %i\r\n", vsense_unscaled);
  cli.println(buf);
  float voltage =
      (float)vsense_unscaled * 0.04174358974; //  103.6 /(4095*2/3.3)
  uint16_t voltage_int = voltage * 100;
  sprintf(buf, "voltage value %i\r\n", voltage_int);
  cli.println(buf);

  return CLI_OK;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
