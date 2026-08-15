/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RN2483_BAUDRATE      57600U
#define RN2483_RESP_TIMEOUT  1000U   /* ms, for the immediate command response */
#define RN2483_TX_TIMEOUT    10000U  /* ms, for "radio_tx_ok" (SF12 frames are slow) */
#define RADIO_PACKET_PREFIX  "VA7FAH "
/* Pause between frames. Observed message spacing is this plus the frame's
   airtime (~1.5 s at SF12), so 1200 ms lands at ~2.5-3 s between messages. */
#define TX_PERIOD_MS         1200U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart1; /* RN2483 LoRa module, PA9/PA10 = D8/D2 */
UART_HandleTypeDef huart2; /* debug console on the ST-LINK VCP, PA2/PA3 */
static uint32_t tx_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void Debug_UART_Init(void);
static void Debug_Log(const char *msg);
static void RN2483_UART_Init(void);
static void RN2483_Flush(void);
static HAL_StatusTypeDef RN2483_ReadLine(char *buf, uint16_t size, uint32_t timeout);
static HAL_StatusTypeDef RN2483_Command(const char *cmd, char *resp, uint16_t size, uint32_t timeout);
static void RN2483_CommandChecked(const char *cmd);
static HAL_StatusTypeDef RN2483_RadioTx(const char *payload);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief USART2 init: debug console over the ST-LINK virtual COM port
  *        (PA2/PA3 are hard-wired to the ST-LINK via SB13/SB14).
  *        The GPIO alternate-function setup is already done in MX_GPIO_Init().
  */
static void Debug_UART_Init(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Print a line on the debug console (CR/LF appended).
  */
static void Debug_Log(const char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 100);
  HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);
}

/**
  * @brief USART1 init (PA9 = TX = D8, PA10 = RX = D2, wired to the RN2483).
  *        Wiring: module RX -> D8 (PA9), module TX -> D2 (PA10).
  */
static void RN2483_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = RN2483_BAUDRATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Discard any pending bytes from the module (e.g. the power-up banner).
  */
static void RN2483_Flush(void)
{
  uint8_t c;
  while (HAL_UART_Receive(&huart1, &c, 1, 50) == HAL_OK)
  {
  }
}

/**
  * @brief Read one CR/LF-terminated line from the module into buf.
  *        The terminator is stripped and buf is always NUL-terminated.
  */
static HAL_StatusTypeDef RN2483_ReadLine(char *buf, uint16_t size, uint32_t timeout)
{
  uint32_t start = HAL_GetTick();
  uint16_t len = 0;

  while ((HAL_GetTick() - start) < timeout)
  {
    uint8_t c;
    if (HAL_UART_Receive(&huart1, &c, 1, 10) != HAL_OK)
    {
      continue;
    }
    if (c == '\r')
    {
      continue;
    }
    if (c == '\n')
    {
      if (len == 0)
      {
        continue; /* ignore stray blank lines */
      }
      buf[len] = '\0';
      return HAL_OK;
    }
    if (len < (size - 1U))
    {
      buf[len++] = (char)c;
    }
  }
  buf[len] = '\0';
  return HAL_TIMEOUT;
}

/**
  * @brief Send a command (CR/LF appended) and read the immediate response line.
  */
static HAL_StatusTypeDef RN2483_Command(const char *cmd, char *resp, uint16_t size, uint32_t timeout)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)cmd, (uint16_t)strlen(cmd), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 100);
  return RN2483_ReadLine(resp, size, timeout);
}

/**
  * @brief Send a command that must answer "ok"; hang in Error_Handler otherwise.
  */
static void RN2483_CommandChecked(const char *cmd)
{
  char resp[32];

  if (RN2483_Command(cmd, resp, sizeof(resp), RN2483_RESP_TIMEOUT) != HAL_OK ||
      strcmp(resp, "ok") != 0)
  {
    Error_Handler();
  }
}

/**
  * @brief Transmit a text payload over the radio ("radio tx <hex>").
  *        Waits for the "ok" ack and then for "radio_tx_ok" (frame on air).
  */
static HAL_StatusTypeDef RN2483_RadioTx(const char *payload)
{
  char cmd[96] = "radio tx ";
  char resp[32];
  size_t pos = strlen(cmd);
  const size_t station_prefix_length = strlen(RADIO_PACKET_PREFIX);
  const size_t payload_length = strlen(payload);
  const size_t transmitted_length = station_prefix_length + payload_length;

  if (payload_length == 0U ||
      pos + (transmitted_length * 2U) >= sizeof(cmd))
  {
    return HAL_ERROR;
  }

  for (size_t i = 0U; i < transmitted_length; i++)
  {
    const uint8_t byte = i < station_prefix_length
                             ? (uint8_t)RADIO_PACKET_PREFIX[i]
                             : (uint8_t)payload[i - station_prefix_length];
    pos += (size_t)sprintf(&cmd[pos], "%02X", byte);
  }

  if (RN2483_Command(cmd, resp, sizeof(resp), RN2483_RESP_TIMEOUT) != HAL_OK ||
      strcmp(resp, "ok") != 0)
  {
    return HAL_ERROR;
  }

  if (RN2483_ReadLine(resp, sizeof(resp), RN2483_TX_TIMEOUT) != HAL_OK ||
      strcmp(resp, "radio_tx_ok") != 0)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
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
  /* USER CODE BEGIN 2 */
  Debug_UART_Init();
  Debug_Log("range-test TX: booting");

  RN2483_UART_Init();

  /* Give the module time to boot, then discard its power-up banner. */
  HAL_Delay(1000);
  RN2483_Flush();

  /* Pause the LoRaWAN stack so raw "radio" commands can be used.
     The reply is the pause duration in ms, not "ok".
     Retries forever; on each failure it probes the module and reports the raw
     bytes received (if any) so wiring vs. baud problems can be told apart. */
  {
    char resp[48];
    char dbg[96];
    uint8_t raw[16];
    uint32_t attempt = 0;

    for (;;)
    {
      if (RN2483_Command("mac pause", resp, sizeof(resp), RN2483_RESP_TIMEOUT) == HAL_OK)
      {
        snprintf(dbg, sizeof(dbg), "RN2483 alive (mac pause -> %s)", resp);
        Debug_Log(dbg);
        break;
      }

      attempt++;
      HAL_UART_Transmit(&huart1, (uint8_t *)"sys get ver\r\n", 13, 100);
      uint16_t n = 0;
      uint32_t t0 = HAL_GetTick();
      while (n < sizeof(raw) && (HAL_GetTick() - t0) < 1000U)
      {
        if (HAL_UART_Receive(&huart1, &raw[n], 1, 10) == HAL_OK)
        {
          n++;
        }
      }

      if (n == 0)
      {
        snprintf(dbg, sizeof(dbg), "attempt %lu: RX totally silent - check TX/RX crossover and module power",
                 (unsigned long)attempt);
      }
      else
      {
        int pos = snprintf(dbg, sizeof(dbg), "attempt %lu: %u raw bytes:", (unsigned long)attempt, n);
        for (uint16_t i = 0; i < n && pos < (int)sizeof(dbg) - 4; i++)
        {
          pos += snprintf(&dbg[pos], sizeof(dbg) - (size_t)pos, " %02X", raw[i]);
        }
      }
      Debug_Log(dbg);
      __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_FEF | UART_CLEAR_OREF | UART_CLEAR_NEF);
    }
  }

  /* Radio settings — the receiver must use the exact same values.
     SF12 / 125 kHz gives maximum range at the cost of ~1.5 s airtime per frame. */
  RN2483_CommandChecked("radio set mod lora");
  RN2483_CommandChecked("radio set freq 433575000"); /* 433 band - matches the antennas */
  RN2483_CommandChecked("radio set sf sf12");
  RN2483_CommandChecked("radio set pwr 14");
  Debug_Log("radio configured: 433.575 MHz, SF12, +14 dBm - transmitting");
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    char msg[32];

    snprintf(msg, sizeof(msg), "PING %lu", (unsigned long)tx_count++);
    if (RN2483_RadioTx(msg) == HAL_OK)
    {
      BSP_LED_Toggle(LED_GREEN); /* toggles on every frame that made it on air */
      Debug_Log(msg);
    }
    else
    {
      Debug_Log("ERROR: radio tx failed");
    }
    HAL_Delay(TX_PERIOD_MS);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
