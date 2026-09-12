/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for STM32G474RET6 FDCAN Inter-Communication
  * This code demonstrates communication between FDCAN1 (Tx)
  * and FDCAN2 (Rx) on a single STM32G474RET6 microcontroller
  * via external MCP2562 CAN transceivers.
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
#include "stdio.h" // Required for printf
#include "string.h" // Required for memset (though less used now for number data)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define GPIOs for debugging LEDs (adjust these based on your CubeMX setup)
#define LD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_14 // Example: User LED 1
#define LD2_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_15 // Example: User LED 2
#define LD_ERROR_GPIO_Port GPIOA
#define LD_ERROR_Pin GPIO_PIN_5 // Example: Onboard LED (Nucleo-G474RE has PA5)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader; // FDCAN Transmit Header structure
FDCAN_RxHeaderTypeDef RxHeader; // FDCAN Receive Header structure

uint8_t TxData[64]; // Transmit data buffer (max 64 bytes for CAN FD)
uint8_t RxData[64]; // Receive data buffer (max 64 bytes for CAN FD)

volatile uint32_t received_message_count = 0; // Counter for successfully received messages

// New global variables for buffering received message from ISR
volatile uint8_t new_message_received_flag = 0;
FDCAN_RxHeaderTypeDef Global_RxHeader;
uint8_t Global_RxData[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void FDCAN2_Filter_Config(void); // Function to configure FDCAN2's message filter
void Error_Handler(void); // Custom error handler
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Retargets the C library printf function to the UART.
  * @param file: file pointer
  * @param ptr: pointer to data buffer
  * @param len: length of data to be sent
  * @retval int: number of bytes sent
  */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}


#define SIG_GPIO_Port	GPIOB
#define SIG_Pin			GPIO_PIN_1
#define MUX3_GPIO_Port	GPIOB
#define MUX3_Pin		GPIO_PIN_10
#define MUX1_GPIO_Port	GPIOA
#define MUX1_Pin		GPIO_PIN_4
#define MUX2_GPIO_Port	GPIOB
#define MUX2_Pin		GPIO_PIN_4
#define MUX0_GPIO_Port	GPIOA
#define MUX0_Pin		GPIO_PIN_10
#define DRST_GPIO_Port	GPIOB
#define DRST_Pin		GPIO_PIN_0

void SetMuxChannel(uint8_t channel) {
  HAL_GPIO_WritePin(MUX0_GPIO_Port, MUX0_Pin, (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX1_GPIO_Port, MUX1_Pin, (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX2_GPIO_Port, MUX2_Pin, (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX3_GPIO_Port, MUX3_Pin, (channel & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  MX_FDCAN2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

    printf("FDCAN Inter-Communication Test Started!\r\n");

    // Configure FDCAN2 filter to accept messages from FDCAN1's ID (0x123)
    // IMPORTANT: Filters must be configured AFTER HAL_FDCAN_Init() but BEFORE HAL_FDCAN_Start()
    FDCAN2_Filter_Config();
    printf("FDCAN2 Filter Configured.\r\n");

    // Start FDCAN peripherals
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        printf("Error: FDCAN1 Start failed!\r\n");
        Error_Handler();
    }
    printf("FDCAN1 Started.\r\n");

    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        printf("Error: FDCAN2 Start failed!\r\n");
        Error_Handler();
    }
    printf("FDCAN2 Started.\r\n");

    // Activate FDCAN2 Rx FIFO 0 New Message Interrupt
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        printf("Error: FDCAN2 Rx FIFO 0 Notification Activation failed!\r\n");
        Error_Handler();
    }
    printf("FDCAN2 Rx FIFO 0 Notification Activated.\r\n");

    // --- Prepare Tx Header for FDCAN1 ---
    TxHeader.Identifier = 0x123;           // Standard ID for FDCAN1 messages
    TxHeader.IdType = FDCAN_STANDARD_ID;   // 11-bit ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; // Data Frame
    // Set DataLength back to 8 bytes for numerical data
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON; // Enable Bit Rate Switching for data phase
    TxHeader.FDFormat = FDCAN_FD_CAN;     // CAN FD format
    TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS; // Optional: Store Tx events
    TxHeader.MessageMarker = 0;            // Optional: Message marker

    // Initialize TxData[0] for incrementing number
    TxData[0] = 0;
    // Clear the rest of the TxData buffer to ensure clean transmission
    memset(&TxData[1], 0, sizeof(TxData) - 1);
    printf("FDCAN1 Tx Header and initial Data prepared with a number.\r\n");


    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(DRST_GPIO_Port, DRST_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        // --- Transmit message from FDCAN1 ---
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
        {
            printf("Error: FDCAN1 Message Transmission failed!\r\n");
            // Check FDCAN error status registers for more details if this error persists
            // For example, you can check hfdcan1.ErrorCode or FDCAN1->PSR (Protocol Status Register)
            Error_Handler();
        }
        else
        {
            // Print the first byte of data for confirmation
            printf("FDCAN1 Transmitted ID: 0x%X, Data[0]: %d\r\n", (unsigned int)TxHeader.Identifier, TxData[0]);
        }

        // Increment first byte of TxData for next transmission to show change
        TxData[0]++;
        if (TxData[0] > 0xFF) TxData[0] = 0; // Wrap around

        // --- Process Received Message in main loop (from ISR) ---
        if (new_message_received_flag == 1)
        {
            printf("FDCAN2 Received Message #%lu:\r\n", received_message_count);
            printf("  ID: 0x%X, DLC: %d, FDFormat: %s, BRS: %s\r\n",
                   (unsigned int)Global_RxHeader.Identifier,
                   (int)Global_RxHeader.DataLength,
                   (Global_RxHeader.FDFormat == FDCAN_FD_CAN) ? "CAN FD" : "Classic CAN",
                   (Global_RxHeader.BitRateSwitch == FDCAN_BRS_ON) ? "ON" : "OFF");
            // Print received data[0] as an integer
            printf("  Data[0]: %d\r\n", Global_RxData[0]);

            // Example: Toggle an LED based on the received data (e.g., if the first byte is even)
            if (Global_RxData[0] % 2 == 0)
            {
                HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin); // Toggle LD1 for even data[0]
            }
            else
            {
                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Toggle LD2 for odd data[0]
            }

            new_message_received_flag = 0; // Clear the flag after processing
        }


        // Small delay between transmissions
        //HAL_Delay(500); // Transmit every 500ms

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        SetMuxChannel(6);

        	GPIO_PinState data = HAL_GPIO_ReadPin(SIG_GPIO_Port, SIG_Pin);

        	if (data == GPIO_PIN_SET){

        		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        		HAL_Delay(2500);
        		HAL_GPIO_WritePin(DRST_GPIO_Port, DRST_Pin, GPIO_PIN_SET);
        		HAL_Delay(2500);
        		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        		HAL_GPIO_WritePin(DRST_GPIO_Port, DRST_Pin, GPIO_PIN_RESET);
        	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
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
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 17;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 16;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 17;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 16;
  hfdcan1.Init.DataTimeSeg2 = 3;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  // Configure Transceiver Delay Compensation (TDC) for FDCAN1
  // TdcOffset = 5, TdcFilter = 0 are typical values for high data rates as per ST examples
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 5, 0) != HAL_OK)
  {
      Error_Handler();
  }
  // Enable TDC
  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 17;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 16;
  hfdcan2.Init.NominalTimeSeg2 = 3;
  hfdcan2.Init.DataPrescaler = 17;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 16;
  hfdcan2.Init.DataTimeSeg2 = 3;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
  // Configure Transceiver Delay Compensation (TDC) for FDCAN2
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, 5, 0) != HAL_OK)
  {
      Error_Handler();
  }
  // Enable TDC
  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE END FDCAN2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  /* Enable clocks for LED GPIOs */
  __HAL_RCC_GPIOC_CLK_ENABLE(); // For LD1_Pin, LD2_Pin
  __HAL_RCC_GPIOA_CLK_ENABLE(); // For LD_ERROR_Pin
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Configure LD1_Pin (PC14) and LD2_Pin (PC15) as Output
  GPIO_InitStruct.Pin = LD1_Pin | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  // Configure LD_ERROR_Pin (PA5) as Output
  GPIO_InitStruct.Pin = LD_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD_ERROR_GPIO_Port, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Configures the FDCAN2 message filter.
  * This filter is set to accept standard ID 0x123.
  * @param None
  * @retval None
  */
static void FDCAN2_Filter_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;             // Filter for 11-bit standard IDs
    sFilterConfig.FilterIndex = 0;                        // Use the first filter element
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;         // Mask filter mode
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Route accepted messages to Rx FIFO 0
    sFilterConfig.FilterID1 = 0x123;                      // Filter ID: accept messages with ID 0x123
    sFilterConfig.FilterID2 = 0x7FF;                      // Filter Mask: all bits must match (0x7FF for standard ID)


    // Configure FDCAN2 filter
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        printf("Error: FDCAN2 Filter Configuration failed!\r\n");
        Error_Handler();
    }

    // Configure global filter to accept all non-matching standard/extended messages into Rx FIFO 0/1,
    // and reject remote frames.
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        printf("Error: FDCAN2 Global Filter Configuration failed!\r\n");
        Error_Handler();
    }
}

/**
  * @brief  Rx FIFO 0 Callback Function.
  * This function is called when a new message is received in Rx FIFO 0.
  * @param  hfdcan: FDCAN handle
  * @param  RxFifo0ITs: RxFifo0 Interrupt flags
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // Check if the interrupt is for a new message in Rx FIFO 0
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        // Ensure this callback is for FDCAN2
        if (hfdcan->Instance == FDCAN2)
        {
            // Retrieve message from Rx FIFO 0
            // Store it in global variables to be processed in the main loop
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Global_RxHeader, Global_RxData) != HAL_OK)
            {
                // In an ISR, avoid printf. Just set error flag or toggle LED if critical.
                Error_Handler(); // Keep this for now, but ideally this would be a very minimal error indication.
            }
            else
            {
                received_message_count++; // Increment counter
                new_message_received_flag = 1; // Set flag to process in main loop
            }

            // Re-activate the notification for the next message
            if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
            {
                Error_Handler(); // Again, ideally very minimal in ISR.
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
  printf("!!! An Error Occurred !!!\r\n");
  __disable_irq();
  while (1)
  {
    // Blink an error LED indefinitely
    HAL_GPIO_TogglePin(LD_ERROR_GPIO_Port, LD_ERROR_Pin);
    HAL_Delay(200);
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
  printf("Wrong parameters value: file %s on line %lu\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
