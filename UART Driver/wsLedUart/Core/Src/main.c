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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void sendLed(uint32_t _ledColor);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // WS LED data that will be sent to the LED. It's packed 4 bits but note that
  // UART is using start and stop bits. There is detailed explanation below.
  // per one bit of the LED color. And it's also inverted, due UART high-idle
  // state. So, before sending the data to the LED, signal must be inverted with
  // CMOS/TTL inverter. I used SN74LS00N with both inputs connected together and
  // with 1k pull-up at it's inputs.
  // How it works actually:
  // Byte array:    11001110   11001110   11001110   10001100   11001110   11001110   11001110   11001110   11001110   11001110   11001110   11001110
  // Start&stop:   S        E S        E S        E S        E S        E S        E S        E S        E S        E S        E S        E S        E
  // MSB->LSB:      01110011   01110011   01110011   00110001   01110011   01110011   01110011   01110011   01110011   01110011   01110011   01110011
  // Inverted       10001100   10001100   10001100   11001110   10001100   10001100   10001100   10001100   10001100   10001100   10001100   10001100
  // Start&Stop:   0        1 0        1 0        1 0        1 0        1 0        1 0        1 0        1 0        1 0        1 0        1 0        1
  // Start&StopInv:1        0 1        0 1        0 1        0 1        0 1        0 1        0 1        0 1        0 1        0 1        0 1        0
  // All:          1100011000 1100011000 1100011000 1110011100 1100011000 1100011000 1100011000 1100011000 1100011000 1100011000 1100011000 1100011000
  // WS Data:      G7:0 G6:0  G5:0 G4:0  G3:0 G2:0  G1:1 G0:1  R7:0 R6:0  R5:0 R4:0  R3:0 R2:0  R1:0 R0:0  B7:0 B6:0  B5:0 B4:0  B3:0 B2:0  B1:0 B0:0
  // This array will show green LED (same as example above)
  //uint8_t wsData[] =
  //{
  //    0b11001110,
  //    0b11001110,
  //    0b11001110,
  //    0b10001100,
  //    0b11001110,
  //    0b11001110,
  //    0b11001110,
  //    0b11001110,
  //    0b11001110,
  //    0b11001110,
  //    0b11001110,
  //    0b11001110,
  //};

  // Enable the DMA for the UART peripheral.
  __HAL_DMA_ENABLE(&hdma_usart1_tx);

  // Enable transmitting data with DAM on UART.
  ATOMIC_SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

  // Now set the DMA and use this approach for setting the LED color. User is still using
  // the RGB888 as input.
  sendLed(0b000000000000000000000001);
  // Let's do the same for one more LED. Be careful here! if the pause between LEDs data
  // is larger then 40us, data will be applied to the LEDs will be reset.
  sendLed(0b000000000000000100000000);

  // Lets add few more colors.
  sendLed(0x010000);
  sendLed(0x010100);
  sendLed(0x010001);
  sendLed(0x000101);
  sendLed(0x010101);

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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 4000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void sendLed(uint32_t _ledColor)
{
    uint8_t wsData[] =
    {
        0b11001110,
        0b11001110,
        0b11001110,
        0b11001110,
        0b11001110,
        0b11001110,
        0b11001110,
        0b11001110,
        0b11001110,
        0b11001110,
        0b11001110,
        0b11001110,
    };

    // First modify the red channel.
    for (int i = 16; i < 24; i++)
    {
        if ((_ledColor >> i) & 1)
        {
            // (i-16) is to shift i from 16 to 24 to 0 to 8. That divided by to is because
            // there are two LED bits in one byte. A then 7 minus all calculated
            // before is to shift and move everything, so the bits that are being modified
            // go from 7 to 3 (see the detailed explanation on how the data is send (how
            // everything works).
            wsData[7 - ((i - 16) / 2)] &= (i & 1) ? ~(0b00000010): ~(0b01000000);
        }
    }

    // Now modify the green channel.
    for (int i = 8; i < 16; i++)
    {
        if ((_ledColor >> i) & 1)
        {
            wsData[3 - ((i - 8) / 2)] &= (i & 1) ? ~(0b00000010): ~(0b01000000);
        }
    }

    // Now modify the blue channel.
    for (int i = 0; i < 8; i++)
    {
        if ((_ledColor >> i) & 1)
        {
            wsData[11 - i/2] &= (i & 1) ? ~(0b00000010): ~(0b01000000);
        }
    }

    // Start the DMA transfer with pre-set UART settings. In this case it's data only for one LED.
    HAL_DMA_Start(&hdma_usart1_tx, (uint32_t)(wsData), (uint32_t)(&(huart1.Instance->DR)), 12);

    // Wait until the data has been sent completely.
    HAL_DMA_PollForTransfer(&hdma_usart1_tx, HAL_DMA_FULL_TRANSFER, 1ULL);
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
