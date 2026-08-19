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
#include <stdbool.h>
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
#define RN2483_RX_WDT_MS     2000U   /* short slices keep serial commands responsive */
#define RN2483_TX_TIMEOUT_MS 10000U
#define SERIAL_COMMAND_SIZE  16U
#define SERIAL_QUEUE_DEPTH   2U
#define SILENCE_LOG_MS       15000U
#define CONTROL_COMMAND_REPEATS 3U
#define ONE_SHOT_COMMAND_REPEATS 1U
#define PING_REPLY_TIMEOUT_MS 12000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart1; /* RN2483 LoRa module, PA9/PA10 = D8/D2 */
static uint32_t rx_count = 0;
static uint8_t serial_rx_byte;
static volatile char serial_input[SERIAL_COMMAND_SIZE];
static volatile uint8_t serial_input_length;
static volatile bool serial_command_discarding;
static volatile bool serial_command_overflow;
static volatile char serial_queue[SERIAL_QUEUE_DEPTH][SERIAL_COMMAND_SIZE];
static volatile uint8_t serial_queue_length[SERIAL_QUEUE_DEPTH];
static volatile uint8_t serial_queue_head;
static volatile uint8_t serial_queue_tail;
static volatile uint8_t serial_queue_count;
static bool ping_waiting;
static uint32_t ping_started_ms;
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
static HAL_StatusTypeDef RN2483_ReadLineInterruptible(char *buf,
                                                      uint16_t size,
                                                      uint32_t timeout);
static HAL_StatusTypeDef RN2483_Command(const char *cmd, char *resp, uint16_t size, uint32_t timeout);
static HAL_StatusTypeDef RN2483_StopReceive(char *detail, uint16_t detail_size);
static void RN2483_CommandChecked(const char *cmd);
static HAL_StatusTypeDef RN2483_TransmitText(const char *text, char *detail, uint16_t detail_size);
static void Serial_Command_Init(void);
static bool Serial_TakeCommand(char *command, uint16_t command_size);
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
static HAL_StatusTypeDef RN2483_ReadLineInternal(char *buf,
                                                 uint16_t size,
                                                 uint32_t timeout,
                                                 bool serial_interruptible)
{
  uint32_t start = HAL_GetTick();
  uint16_t len = 0;

  while ((HAL_GetTick() - start) < timeout)
  {
    if (serial_interruptible && serial_queue_count > 0U && len == 0U)
    {
      buf[len] = '\0';
      return HAL_BUSY;
    }

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

static HAL_StatusTypeDef RN2483_ReadLine(char *buf,
                                         uint16_t size,
                                         uint32_t timeout)
{
  return RN2483_ReadLineInternal(buf, size, timeout, false);
}

static HAL_StatusTypeDef RN2483_ReadLineInterruptible(char *buf,
                                                      uint16_t size,
                                                      uint32_t timeout)
{
  return RN2483_ReadLineInternal(buf, size, timeout, true);
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
  * @brief Stop continuous RX and wait for this command's explicit "ok".
  *        Pending radio_rx/radio_err lines belong to the old receive command
  *        and must not be mistaken for the rxstop response.
  */
static HAL_StatusTypeDef RN2483_StopReceive(char *detail, uint16_t detail_size)
{
  static const char command[] = "radio rxstop\r\n";
  char line[80];
  const uint32_t started_ms = HAL_GetTick();

  if (detail == NULL || detail_size == 0U)
  {
    return HAL_ERROR;
  }
  detail[0] = '\0';

  if (HAL_UART_Transmit(&huart1, (uint8_t *)command,
                        (uint16_t)(sizeof(command) - 1U), 100U) != HAL_OK)
  {
    snprintf(detail, detail_size, "rxstop_write_failed");
    return HAL_ERROR;
  }

  while ((HAL_GetTick() - started_ms) < RN2483_RESP_TIMEOUT)
  {
    const uint32_t elapsed_ms = HAL_GetTick() - started_ms;
    const uint32_t remaining_ms = RN2483_RESP_TIMEOUT - elapsed_ms;
    if (RN2483_ReadLine(line, sizeof(line), remaining_ms) != HAL_OK)
    {
      break;
    }

    if (strcmp(line, "ok") == 0)
    {
      RN2483_Flush();
      snprintf(detail, detail_size, "ok");
      return HAL_OK;
    }

    if (strcmp(line, "radio_err") == 0 ||
        strncmp(line, "radio_rx", 8U) == 0 ||
        strcmp(line, "busy") == 0)
    {
      continue;
    }

    snprintf(detail, detail_size, "rxstop_%.20s", line);
    return HAL_ERROR;
  }

  snprintf(detail, detail_size, "rxstop_timeout");
  return HAL_TIMEOUT;
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
  * @brief Send an ASCII payload with the RN2483 raw-LoRa radio command.
  *        The immediate "ok" and final "radio_tx_ok" are both required.
  */
static HAL_StatusTypeDef RN2483_TransmitText(const char *text,
                                             char *detail,
                                             uint16_t detail_size)
{
  static const char hex_digits[] = "0123456789ABCDEF";
  char command[80] = "radio tx ";
  const size_t prefix_length = strlen(command);
  const size_t text_length = strlen(text);

  if (detail == NULL || detail_size == 0U)
  {
    return HAL_ERROR;
  }
  detail[0] = '\0';

  if (text_length == 0U ||
      prefix_length + (text_length * 2U) >= sizeof(command))
  {
    snprintf(detail, detail_size, "payload_too_long");
    return HAL_ERROR;
  }

  for (size_t i = 0; i < text_length; i++)
  {
    const uint8_t byte = (uint8_t)text[i];
    command[prefix_length + (i * 2U)] = hex_digits[byte >> 4];
    command[prefix_length + (i * 2U) + 1U] = hex_digits[byte & 0x0FU];
  }
  command[prefix_length + (text_length * 2U)] = '\0';

  if (RN2483_Command(command, detail, detail_size,
                     RN2483_RESP_TIMEOUT) != HAL_OK)
  {
    snprintf(detail, detail_size, "immediate_timeout");
    return HAL_TIMEOUT;
  }
  if (strcmp(detail, "ok") != 0)
  {
    return HAL_ERROR;
  }

  if (RN2483_ReadLine(detail, detail_size,
                      RN2483_TX_TIMEOUT_MS) != HAL_OK)
  {
    snprintf(detail, detail_size, "transmit_timeout");
    return HAL_TIMEOUT;
  }
  return strcmp(detail, "radio_tx_ok") == 0 ? HAL_OK : HAL_ERROR;
}

/**
  * @brief Arm interrupt-driven reception on the ST-Link VCP (USART2).
  */
static void Serial_Command_Init(void)
{
  serial_input_length = 0U;
  serial_command_discarding = false;
  serial_command_overflow = false;
  serial_queue_head = 0U;
  serial_queue_tail = 0U;
  serial_queue_count = 0U;

  HAL_NVIC_SetPriority(USART2_IRQn, 1U, 0U);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  if (HAL_UART_Receive_IT(&huart2, &serial_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Copy one complete serial line out of the interrupt-owned buffer.
  */
static bool Serial_TakeCommand(char *command, uint16_t command_size)
{
  if (command == NULL || command_size == 0U || serial_queue_count == 0U)
  {
    return false;
  }

  __disable_irq();
  const uint8_t slot = serial_queue_tail;
  uint8_t length = serial_queue_length[slot];
  if (length >= command_size)
  {
    length = (uint8_t)(command_size - 1U);
  }
  for (uint8_t index = 0U; index < length; index++)
  {
    command[index] = serial_queue[slot][index];
  }
  command[length] = '\0';
  serial_queue_tail = (uint8_t)((serial_queue_tail + 1U) % SERIAL_QUEUE_DEPTH);
  serial_queue_count--;
  __enable_irq();
  return true;
}

/**
  * @brief Assemble CR/LF-terminated WebSerial commands without blocking the
  *        RN2483 receive wait in the main loop.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart2)
  {
    return;
  }

  const uint8_t byte = serial_rx_byte;
  if (byte == '\n')
  {
    if (serial_command_discarding)
    {
      serial_command_discarding = false;
      serial_command_overflow = true;
      serial_input_length = 0U;
    }
    else if (serial_input_length > 0U)
    {
      if (serial_queue_count < SERIAL_QUEUE_DEPTH)
      {
        const uint8_t slot = serial_queue_head;
        for (uint8_t index = 0U; index < serial_input_length; index++)
        {
          serial_queue[slot][index] = serial_input[index];
        }
        serial_queue[slot][serial_input_length] = '\0';
        serial_queue_length[slot] = serial_input_length;
        serial_queue_head =
            (uint8_t)((serial_queue_head + 1U) % SERIAL_QUEUE_DEPTH);
        serial_queue_count++;
      }
      else
      {
        serial_command_overflow = true;
      }
      serial_input_length = 0U;
    }
  }
  else if (byte >= 0x20U && byte <= 0x7EU && !serial_command_discarding)
  {
    if (serial_input_length < (SERIAL_COMMAND_SIZE - 1U))
    {
      serial_input[serial_input_length++] = (char)byte;
    }
    else
    {
      serial_command_discarding = true;
      serial_input_length = 0U;
    }
  }

  (void)HAL_UART_Receive_IT(&huart2, &serial_rx_byte, 1U);
}

/**
  * @brief Recover VCP reception after an overrun/noise/framing error.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    (void)HAL_UART_Receive_IT(&huart2, &serial_rx_byte, 1U);
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
  Serial_Command_Init();
  Debug_Log("range-test RX: booting");

  RN2483_UART_Init();

  /* The RN2483 is powered separately and can still be finishing a receive
     window after this MCU is reflashed/reset.  Wait through that window,
     then discard its asynchronous radio_err/power-up output. */
  HAL_Delay(RN2483_RX_WDT_MS + 250U);
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
      const HAL_StatusTypeDef pause_status =
          RN2483_Command("mac pause", resp, sizeof(resp), RN2483_RESP_TIMEOUT);
      bool pause_reply_valid =
          (pause_status == HAL_OK && resp[0] != '\0' && strcmp(resp, "0") != 0);

      for (size_t index = 0U; pause_reply_valid && resp[index] != '\0'; index++)
      {
        if (resp[index] < '0' || resp[index] > '9')
        {
          pause_reply_valid = false;
        }
      }

      if (pause_reply_valid)
      {
        snprintf(dbg, sizeof(dbg), "RN2483 alive (mac pause -> %s)", resp);
        Debug_Log(dbg);
        if (RN2483_Command("sys get ver", resp, sizeof(resp), RN2483_RESP_TIMEOUT) == HAL_OK)
        {
          snprintf(dbg, sizeof(dbg), "module firmware: %s", resp);
          Debug_Log(dbg);
        }

        /* A module-side RX session survives an MCU-only reset.  Force it idle
           after MAC pause has succeeded, then drain the rxstop outcome before
           beginning the known-good radio configuration sequence. */
        (void)RN2483_StopReceive(resp, sizeof(resp));
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
  RN2483_CommandChecked("radio set bw 125");
  RN2483_CommandChecked("radio set cr 4/5");
  RN2483_CommandChecked("radio set crc on");
  RN2483_CommandChecked("radio set iqi off");
  RN2483_CommandChecked("radio set prlen 8");
  RN2483_CommandChecked("radio set sync 34");
  RN2483_CommandChecked("radio set pwr 14");
  /* Watchdog: each listen window ends with radio_err after this many ms,
     so the loop stays alive and can re-arm even if nothing is heard. */
  RN2483_CommandChecked("radio set wdt 2000");
  Debug_Log("radio configured: 433.575 MHz, SF12/BW125/CR4/5, sync 34");
  Debug_Log("bridge serial ready: PUMP_TOGGLE, PUMP_RUN_6_5, LED_ON/OFF, or PING");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    char resp[32];
    char line[160];
    char out[160];
    char serial_line[SERIAL_COMMAND_SIZE];
    static uint32_t last_silence_log_ms = 0U;

    if (serial_command_overflow)
    {
      __disable_irq();
      serial_command_overflow = false;
      __enable_irq();
      Debug_Log("BRIDGE SERIAL ERROR input dropped");
    }

    if (ping_waiting &&
        (HAL_GetTick() - ping_started_ms) >= PING_REPLY_TIMEOUT_MS)
    {
      const uint32_t elapsed_ms = HAL_GetTick() - ping_started_ms;
      ping_waiting = false;
      snprintf(out, sizeof(out), "BRIDGE PING TIMEOUT elapsed_ms=%lu",
               (unsigned long)elapsed_ms);
      Debug_Log(out);
    }

    /* A queued VCP command interrupts the receive wait. Confirm that rxstop
       itself answered "ok" before removing the command from the queue. */
    if (serial_queue_count > 0U)
    {
      char stop_detail[32];
      if (RN2483_StopReceive(stop_detail, sizeof(stop_detail)) != HAL_OK)
      {
        snprintf(out, sizeof(out), "BRIDGE RADIO STOP FAILED (%s)",
                 stop_detail);
        Debug_Log(out);
        continue;
      }
    }

    /* A complete VCP line is handled only when the module is idle and can
       safely switch from RX to TX. */
    if (Serial_TakeCommand(serial_line, sizeof(serial_line)))
    {
      const bool is_pump_toggle = strcmp(serial_line, "PUMP_TOGGLE") == 0;
      const bool is_pump_run = strcmp(serial_line, "PUMP_RUN_6_5") == 0;
      if (is_pump_toggle ||
          is_pump_run ||
          strcmp(serial_line, "LED_ON") == 0 ||
          strcmp(serial_line, "LED_OFF") == 0 ||
          strcmp(serial_line, "PING") == 0)
      {
        char detail[32];
        HAL_StatusTypeDef transmit_status = HAL_OK;
        uint8_t transmissions = 0U;
        const bool is_ping = strcmp(serial_line, "PING") == 0;
        const bool is_one_shot = is_ping || is_pump_toggle || is_pump_run;
        const uint8_t required_transmissions =
            is_one_shot ? ONE_SHOT_COMMAND_REPEATS
                        : CONTROL_COMMAND_REPEATS;

        if (is_ping && ping_waiting)
        {
          Debug_Log("BRIDGE PING BUSY");
          continue;
        }

        snprintf(out, sizeof(out), "BRIDGE SERIAL RX len=%u \"%s\"",
                 (unsigned int)strlen(serial_line), serial_line);
        Debug_Log(out);

        if (is_ping)
        {
          ping_started_ms = HAL_GetTick();
        }

        /* LED commands are idempotent and repeated for link margin. Toggle,
           fixed-duration pump run, and PING commands are sent exactly once. */
        for (uint8_t attempt = 0U;
             attempt < required_transmissions;
             attempt++)
        {
          transmit_status = RN2483_TransmitText(serial_line,
                                                detail,
                                                sizeof(detail));
          if (transmit_status != HAL_OK)
          {
            break;
          }
          transmissions++;
          if (attempt + 1U < required_transmissions)
          {
            HAL_Delay(250U);
          }
        }

        if (transmit_status == HAL_OK &&
            transmissions == required_transmissions)
        {
          snprintf(out, sizeof(out), "BRIDGE RADIO TX %s OK repeats=%u",
                   serial_line, transmissions);
          Debug_Log(out);
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          if (is_ping)
          {
            ping_waiting = true;
          }
        }
        else
        {
          snprintf(out, sizeof(out),
                   "BRIDGE RADIO TX %s FAILED attempt=%u (%s)",
                   serial_line, (unsigned int)(transmissions + 1U), detail);
          Debug_Log(out);
          RN2483_Flush();
        }
      }
      else
      {
        int position = snprintf(out, sizeof(out),
                                "BRIDGE SERIAL ERROR unknown len=%u hex=",
                                (unsigned int)strlen(serial_line));
        for (size_t index = 0U;
             index < strlen(serial_line) && position < (int)sizeof(out) - 3;
             index++)
        {
          position += snprintf(&out[position], sizeof(out) - (size_t)position,
                               "%02X", (uint8_t)serial_line[index]);
        }
        Debug_Log(out);
      }
      continue;
    }

    /* Arm one receive window (0 = until a packet or the watchdog fires). */
    const HAL_StatusTypeDef arm_status =
        RN2483_Command("radio rx 0", resp, sizeof(resp), RN2483_RESP_TIMEOUT);
    if (arm_status != HAL_OK || strcmp(resp, "ok") != 0)
    {
      /* Refusal usually means a stale rx window is still active (e.g. after an
         MCU reset mid-listen) and command/response pairing has drifted. Force
         the radio idle, then drain every queued line so responses resync. */
      snprintf(out, sizeof(out), "'radio rx 0' refused (%s) - resyncing",
               arm_status == HAL_OK ? resp : "timeout");
      Debug_Log(out);
      (void)RN2483_StopReceive(resp, sizeof(resp));
      continue;
    }

    /* Wait for a packet, the watchdog, or a host command. A host command stops
       this receive window instead of inheriting its remaining queue delay. */
    const HAL_StatusTypeDef receive_status =
        RN2483_ReadLineInterruptible(line,
                                     sizeof(line),
                                     RN2483_RX_WDT_MS + 2000U);
    if (receive_status == HAL_BUSY)
    {
      /* The next loop iteration stops RX before dequeuing the VCP command. */
      continue;
    }
    if (receive_status != HAL_OK)
    {
      Debug_Log("(module went quiet, re-arming)");
      (void)RN2483_StopReceive(resp, sizeof(resp));
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
      const uint32_t packet_received_ms = HAL_GetTick();

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
      if (strcmp(text, "PONG") == 0)
      {
        if (ping_waiting)
        {
          ping_waiting = false;
          snprintf(out, sizeof(out),
                   "BRIDGE PING OK rtt_ms=%lu snr=%s rssi=%s",
                   (unsigned long)(packet_received_ms - ping_started_ms),
                   snr,
                   rssi);
          Debug_Log(out);
        }
        else
        {
          Debug_Log("BRIDGE PING UNEXPECTED PONG");
        }
      }
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
    else if (strcmp(line, "radio_err") == 0)
    {
      const uint32_t now = HAL_GetTick();
      if ((now - last_silence_log_ms) >= SILENCE_LOG_MS)
      {
        Debug_Log("(nothing heard recently, still listening)");
        last_silence_log_ms = now;
      }
    }
    else
    {
      snprintf(out, sizeof(out), "unexpected from module: %.135s", line);
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
