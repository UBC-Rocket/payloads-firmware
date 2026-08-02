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
#define RN2483_RX_WDT_MS     15000U  /* radio watchdog: one listen window */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart1; /* RN2483 LoRa module, PA9/PA10 = D8/D2 */
static uint32_t rx_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Debug_Log(const char *msg);
static void RN2483_UART_Init(void);
static void RN2483_Flush(void);
static HAL_StatusTypeDef RN2483_ReadLine(char *buf, uint16_t size, uint32_t timeout);
static HAL_StatusTypeDef RN2483_Command(const char *cmd, char *resp, uint16_t size, uint32_t timeout);
static void RN2483_CommandChecked(const char *cmd);
static int Hex_Nibble(char c);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Print a line on the debug console (ST-LINK VCP, CR/LF appended).
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

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = RN2483_BAUDRATE;
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
    Debug_Log("ERROR: radio config command rejected:");
    Debug_Log(cmd);
    Error_Handler();
  }
}

/**
  * @brief Hex digit -> value, or -1 if not a hex digit.
  */
static int Hex_Nibble(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Debug_Log("range-test RX: booting");

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
        if (RN2483_Command("sys get ver", resp, sizeof(resp), RN2483_RESP_TIMEOUT) == HAL_OK)
        {
          snprintf(dbg, sizeof(dbg), "module firmware: %s", resp);
          Debug_Log(dbg);
        }
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
    }
  }

  /* Radio settings - MUST match the transmitter exactly. */
  RN2483_CommandChecked("radio set mod lora");
  RN2483_CommandChecked("radio set freq 433575000"); /* 433 band - matches the antennas */
  RN2483_CommandChecked("radio set sf sf12");
  /* Watchdog: each listen window ends with radio_err after this many ms,
     so the loop stays alive and can re-arm even if nothing is heard. */
  RN2483_CommandChecked("radio set wdt 15000");
  Debug_Log("radio configured: 433.575 MHz, SF12 - listening");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    char resp[32];
    char line[160];
    char out[160];

    /* Arm one receive window (0 = until a packet or the watchdog fires). */
    if (RN2483_Command("radio rx 0", resp, sizeof(resp), RN2483_RESP_TIMEOUT) != HAL_OK ||
        strcmp(resp, "ok") != 0)
    {
      /* Refusal usually means a stale rx window is still active (e.g. after an
         MCU reset mid-listen) and command/response pairing has drifted. Force
         the radio idle, then drain every queued line so responses resync. */
      Debug_Log("'radio rx 0' refused - resyncing");
      RN2483_Command("radio rxstop", resp, sizeof(resp), RN2483_RESP_TIMEOUT);
      HAL_Delay(200);
      RN2483_Flush();
      continue;
    }

    /* Wait for the outcome of this window: a packet or the watchdog. */
    if (RN2483_ReadLine(line, sizeof(line), RN2483_RX_WDT_MS + 2000U) != HAL_OK)
    {
      Debug_Log("(module went quiet, re-arming)");
      continue;
    }

    if (strncmp(line, "radio_rx", 8) == 0)
    {
      /* Format: "radio_rx  <hex payload>" - decode the hex into text. */
      const char *hex = &line[8];
      while (*hex == ' ')
      {
        hex++;
      }

      char text[64];
      uint16_t tlen = 0;
      while (hex[0] != '\0' && hex[1] != '\0' && tlen < sizeof(text) - 1U)
      {
        int hi = Hex_Nibble(hex[0]);
        int lo = Hex_Nibble(hex[1]);
        if (hi < 0 || lo < 0)
        {
          break;
        }
        char c = (char)((hi << 4) | lo);
        text[tlen++] = (c >= 0x20 && c < 0x7F) ? c : '.';
        hex += 2;
      }
      text[tlen] = '\0';

      /* Signal quality of this packet - the actual range-test measurement.
         SNR = signal vs noise floor (SF12 decodes down to -20 dB);
         RSSI = absolute received power in dBm (needs module firmware >= 1.0.5,
         falls back to "?" on older firmware). */
      char snr[16] = "?";
      char rssi[16] = "?";
      RN2483_Command("radio get snr", snr, sizeof(snr), RN2483_RESP_TIMEOUT);
      RN2483_Command("radio get rssi", rssi, sizeof(rssi), RN2483_RESP_TIMEOUT);
      if (!(rssi[0] == '-' || (rssi[0] >= '0' && rssi[0] <= '9')))
      {
        strcpy(rssi, "?");
      }

      rx_count++;
      snprintf(out, sizeof(out), "RX %lu: \"%s\" (snr %s, rssi %s)",
               (unsigned long)rx_count, text, snr, rssi);
      Debug_Log(out);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
    else if (strcmp(line, "radio_err") == 0)
    {
      Debug_Log("(nothing heard in this window, still listening)");
    }
    else
    {
      snprintf(out, sizeof(out), "unexpected from module: %s", line);
      Debug_Log(out);
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
