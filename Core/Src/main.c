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
#include <string.h>
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ISO 7816 Communication Parameters
// For 3.6MHz clock: 3,600,000 / 372 = 9677 baud (not standard 9600!)
// ETU (Elementary Time Unit) = 1/baud = 103.3 microseconds
// NOTE: If ATR fails, try these alternatives:
//   BIT_DELAY_US = 104 (standard for 3.57MHz, 9600 baud)
//   BIT_DELAY_US = 100 (for higher baud)
//   BIT_DELAY_US = 110 (for lower baud)
#define SIM_BAUDRATE 9600  // Adjusted for 3.6MHz clock (was 9600 for 3.57MHz)
#define BIT_DELAY_US 95    // Bit period in microseconds

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t atr_buffer[33];  // ATR buffer (max 33 bytes)
uint8_t atr_length = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void DWT_Init(void);
void SIM_SetOutput(void);
void SIM_FindTimingOffset(void);
uint8_t SIM_ReadATR_Simple(uint8_t *atr, uint8_t *len);
void SIM_ReadICCID(char *iccid);
uint8_t SIM_ReadOneByte(void);
void SIM_ReadBytes(uint8_t *buf, uint8_t count);

// SIM Card Communication Functions
uint8_t SIM_Init(void);
uint8_t SIM_ReceiveByte(void);
void SIM_SendByte(uint8_t data);
uint8_t SIM_SendAPDU(uint8_t *cmd, uint8_t cmd_len, uint8_t *response, uint8_t *resp_len);
uint8_t SIM_GetResponse(uint8_t len, uint8_t *data);
void SIM_ReadIMSI(char *imsi, uint8_t sim_num);
void delay_us(uint32_t us);

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
  MX_USART1_UART_Init();
//   MX_I2C1_Init();
  DWT_Init();  // Initialize DWT cycle counter for precise µs delays
  /* USER CODE BEGIN 2 */
  
  char msg[100];
  
  // Initialize SIM card interface
  sprintf(msg, "\r\nSIMPOOL STM32F429\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  
  SIM_Init();
  uint8_t atr[33];
  uint8_t atr_len = 0;
  SIM_ReadATR_Simple(atr, &atr_len);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Read IMSI and ICCID
    char imsi[20] = {0};
    char iccid[25] = {0};
    
    SIM_ReadIMSI(imsi, 0);
    sprintf(msg, "IMSI: %s\r\n", imsi);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
    SIM_ReadICCID(iccid);
    sprintf(msg, "ICCID: %s\r\n", iccid);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
    HAL_Delay(5000);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SIM_IO_GPIO_Port, SIM_IO_Pin, GPIO_PIN_SET);  // I/O idle HIGH
  HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_SET);  // RST idle HIGH

  /*Configure GPIO pin : SIM_IO_Pin (PE13 - Bidirectional I/O) */
  GPIO_InitStruct.Pin = SIM_IO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // Open-drain for bidirectional
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // No pull (external pull-up on hardware)
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SIM_IO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SIM_RST_Pin (PE15 - Reset control) */
  GPIO_InitStruct.Pin = SIM_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SIM_RST_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

// ============================================================================
// SIM Card Communication Functions
// ============================================================================

// DWT (Data Watchpoint and Trace) cycle counter for precise delays
#define DWT_CONTROL ((volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  ((volatile uint32_t *)0xE0001004)
#define SCB_DEMCR   ((volatile uint32_t *)0xE000EDFC)

// Initialize DWT cycle counter
void DWT_Init(void) {
    *SCB_DEMCR |= 0x01000000;  // Enable DWT
    *DWT_CYCCNT = 0;            // Reset counter
    *DWT_CONTROL |= 1;          // Enable counter
}

// Precise microsecond delay using DWT cycle counter
void delay_us(uint32_t us) {
    uint32_t cycles = (SystemCoreClock / 1000000) * us;
    uint32_t start = *DWT_CYCCNT;
    while ((*DWT_CYCCNT - start) < cycles);
}

// Set IO line as output (for writing to SIM)
// With open-drain, we keep it in output mode always
void SIM_SetOutput(void) {
    // Pin already configured as open-drain in MX_GPIO_Init()
    // Just ensure it's released (HIGH = high-impedance)
    HAL_GPIO_WritePin(SIM_IO_GPIO_Port, SIM_IO_Pin, GPIO_PIN_SET);
}

// Set IO line ready for input (for reading from SIM)
// With open-drain, we release the pin (write HIGH) and read
void SIM_SetInput(void) {
    // With open-drain: writing HIGH releases the line
    // External pull-up or SIM can then control the line
    HAL_GPIO_WritePin(SIM_IO_GPIO_Port, SIM_IO_Pin, GPIO_PIN_SET);
}

// Write bit to IO line (open-drain: 0=pull low, 1=release)
void SIM_WriteBit(uint8_t bit) {
    if (bit) {
        HAL_GPIO_WritePin(SIM_IO_GPIO_Port, SIM_IO_Pin, GPIO_PIN_SET);  // Release (high-Z)
    } else {
        HAL_GPIO_WritePin(SIM_IO_GPIO_Port, SIM_IO_Pin, GPIO_PIN_RESET);  // Pull low
    }
}

// Read bit from IO line (can read even in output open-drain mode)
uint8_t SIM_ReadBit(void) {
    return HAL_GPIO_ReadPin(SIM_IO_GPIO_Port, SIM_IO_Pin);
}

// Send a byte using ISO 7816-3 protocol (LSB first, even parity)
void SIM_SendByte(uint8_t data) {
    uint8_t parity = 0;
    
    SIM_SetOutput();
    
    // Start bit (LOW)
    SIM_WriteBit(0);
    delay_us(BIT_DELAY_US);
    
    // Send LSB first (ISO 7816-3)
    for (int i = 0; i < 8; i++) {
        uint8_t bit = (data >> i) & 1;
        SIM_WriteBit(bit);
        parity ^= bit;
        delay_us(BIT_DELAY_US);
    }
    
    // Even parity bit
    SIM_WriteBit(parity);
    delay_us(BIT_DELAY_US);
    
    // Guard time (2 stop bits)
    SIM_WriteBit(1);
    delay_us(BIT_DELAY_US * 2);
}
uint8_t SIM_ReceiveByte(void) {
    return SIM_ReadOneByte();
}



// Send APDU command and receive response
uint8_t SIM_SendAPDU(uint8_t *cmd, uint8_t cmd_len, uint8_t *response, uint8_t *resp_len) {
    char msg[80];
    *resp_len = 0;
    
    // Debug: show command
    sprintf(msg, "  CMD: ");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    for (int i = 0; i < cmd_len; i++) {
        sprintf(msg, "%02X ", cmd[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    sprintf(msg, "-> ");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
    SIM_SetOutput();
    
    // Send command header (CLA INS P1 P2 P3)
    for (uint8_t i = 0; i < 5 && i < cmd_len; i++) {
        SIM_SendByte(cmd[i]);
    }
    
    SIM_SetInput();
    
    // Wait for procedure byte
    uint8_t proc_byte = SIM_ReceiveByte();
    sprintf(msg, "%02X ", proc_byte);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
    if (proc_byte == 0xFF) {
        sprintf(msg, "(no response)\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return 0;
    }
    
    // Handle NULL byte (0x60) - SIM needs more time
    while (proc_byte == 0x60) {
        HAL_Delay(10);
        proc_byte = SIM_ReceiveByte();
        sprintf(msg, "%02X ", proc_byte);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    
    // If ACK (INS echoed)
    if (proc_byte == cmd[1]) {
        uint8_t ins = cmd[1];
        uint8_t p3 = cmd[4];
        uint8_t sw1 = 0, sw2 = 0;
        
        // READ commands (B0=READ BINARY, B2=READ RECORD, C0=GET RESPONSE): receive data
        if (ins == 0xB0 || ins == 0xB2 || ins == 0xC0) {
            // Read P3 data bytes + 2 status bytes together
            uint8_t total = p3 + 2;
            SIM_ReadBytes(response, total);
            
            // Print data bytes
            for (int i = 0; i < p3; i++) {
                sprintf(msg, "%02X ", response[i]);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
            *resp_len = p3;
            
            sw1 = response[p3];
            sw2 = response[p3+1];
            sprintf(msg, "[%02X%02X]", sw1, sw2);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        } 
        else {
            // WRITE commands with data (cmd_len > 5): send data
            if (cmd_len > 5) {
                SIM_SetOutput();
                for (uint8_t i = 5; i < cmd_len; i++) {
                    SIM_SendByte(cmd[i]);
                }
                SIM_SetInput();
            }
            
            // Read status words (SW1 SW2)
            sw1 = SIM_ReceiveByte();
            sw2 = SIM_ReceiveByte();
            sprintf(msg, "[%02X%02X]", sw1, sw2);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            
            response[0] = sw1;
            response[1] = sw2;
            *resp_len = 2;
        }
        
        // If SW1=0x9F, there's data to read with GET RESPONSE
        if (sw1 == 0x9F) {
            uint8_t data_len = sw2;
            // Send GET RESPONSE
            SIM_SetOutput();
            SIM_SendByte(0xA0);  // CLA
            SIM_SendByte(0xC0);  // INS = GET RESPONSE
            SIM_SendByte(0x00);  // P1
            SIM_SendByte(0x00);  // P2
            SIM_SendByte(data_len);  // Le
            SIM_SetInput();
            
            uint8_t ack = SIM_ReceiveByte();
            if (ack == 0xC0) {
                // Read data bytes
                for (int i = 0; i < data_len; i++) {
                    response[i] = SIM_ReceiveByte();
                }
                *resp_len = data_len;
                // Read final status
                sw1 = SIM_ReceiveByte();
                sw2 = SIM_ReceiveByte();
            }
        }
    } else if ((proc_byte & 0xF0) == 0x90 || (proc_byte & 0xF0) == 0x60) {
        // Status word directly
        uint8_t sw2 = SIM_ReceiveByte();
        response[0] = proc_byte;
        response[1] = sw2;
        *resp_len = 2;
        sprintf(msg, "%02X ", sw2);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    
    sprintf(msg, "\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
    return (*resp_len > 0) ? 1 : 0;
}

// Simple GET RESPONSE command
uint8_t SIM_GetResponse(uint8_t len, uint8_t *data) {
    char msg[60];
    
    SIM_SetOutput();
    SIM_SendByte(0xA0);  // CLA
    SIM_SendByte(0xC0);  // INS = GET RESPONSE
    SIM_SendByte(0x00);  // P1
    SIM_SendByte(0x00);  // P2
    SIM_SendByte(len);   // Le
    
    SIM_SetInput();
    
    uint8_t proc = SIM_ReceiveByte();
    if (proc == 0xC0) {
        sprintf(msg, "  GET_RSP(%d): ", len);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        
        for (int i = 0; i < len; i++) {
            data[i] = SIM_ReceiveByte();
            sprintf(msg, "%02X ", data[i]);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
        // Read SW1 SW2
        SIM_ReceiveByte();
        SIM_ReceiveByte();
        
        sprintf(msg, "\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return 1;
    }
    return 0;
}

// Read IMSI from SIM card
void SIM_ReadIMSI(char *imsi, uint8_t sim_num) {
    uint8_t response[64];
    uint8_t resp_len;
    (void)sim_num;
    
    strcpy(imsi, "ERROR");
    
    // Select MF (3F00)
    uint8_t select_mf[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x3F, 0x00};
    SIM_SendAPDU(select_mf, 7, response, &resp_len);
    if (resp_len >= 2 && response[0] == 0x9F) {
        SIM_GetResponse(response[1], response);
    }
    HAL_Delay(20);
    
    // Select DF_GSM (7F20)
    uint8_t select_gsm[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x7F, 0x20};
    SIM_SendAPDU(select_gsm, 7, response, &resp_len);
    if (resp_len >= 2 && response[0] == 0x9F) {
        SIM_GetResponse(response[1], response);
    }
    HAL_Delay(20);
    
    // Select EF_IMSI (6F07)
    uint8_t select_imsi[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x6F, 0x07};
    SIM_SendAPDU(select_imsi, 7, response, &resp_len);
    if (resp_len >= 2 && response[0] == 0x9F) {
        SIM_GetResponse(response[1], response);
    }
    HAL_Delay(20);
    
    // Read Binary (9 bytes)
    uint8_t read_bin[] = {0xA0, 0xB0, 0x00, 0x00, 0x09};
    SIM_SendAPDU(read_bin, 5, response, &resp_len);
    
    // Check if we got IMSI data
    if (resp_len >= 9 && response[0] > 0 && response[0] <= 8) {
        int idx = 0;
        // First digit in lower nibble of byte 1
        uint8_t d = response[1] & 0x0F;
        if (d <= 9) imsi[idx++] = '0' + d;
        
        // Remaining digits
        for (int i = 2; i <= response[0] && i < 9; i++) {
            d = response[i] & 0x0F;
            if (d <= 9) imsi[idx++] = '0' + d;
            d = (response[i] >> 4) & 0x0F;
            if (d <= 9) imsi[idx++] = '0' + d;
        }
        imsi[idx] = '\0';
    }
}


// Timing offset and inversion flag
static int g_timing_offset = 0;
static uint8_t g_invert_bits = 0;

// Wait for start bit and read one byte
// Read a byte by waiting for start bit
uint8_t SIM_ReadOneByte(void) {
    uint8_t byte = 0;
    
    // Wait for idle (HIGH) then start bit (LOW)
    uint32_t timeout = 50000;
    while (SIM_ReadBit() == 0 && timeout--) delay_us(1);
    if (timeout == 0) return 0xFF;
    
    timeout = 50000;
    while (SIM_ReadBit() != 0 && timeout--) delay_us(1);
    if (timeout == 0) return 0xFF;
    
    // Wait 1.5 bit times to center on first data bit
    delay_us(BIT_DELAY_US + (BIT_DELAY_US / 2));
    
    // Read 8 data bits (LSB first)
    for (int i = 0; i < 8; i++) {
        byte |= (SIM_ReadBit() << i);
        delay_us(BIT_DELAY_US);
    }
    
    // Skip parity
    delay_us(BIT_DELAY_US);
    
    return byte;
}

// Read multiple consecutive bytes - just call SIM_ReadOneByte for each
void SIM_ReadBytes(uint8_t *buf, uint8_t count) {
    for (int i = 0; i < count; i++) {
        buf[i] = SIM_ReadOneByte();
    }
}

// Verify timing by reading first ATR byte
void SIM_FindTimingOffset(void) {
    char msg[100];
    
    // Reset SIM
    HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_SET);
    
    SIM_SetInput();
    
    uint8_t byte = SIM_ReadOneByte();
    
    sprintf(msg, "First byte: 0x%02X (%s)\r\n", byte, 
            (byte == 0x3B) ? "Direct OK" : 
            (byte == 0x3F) ? "Inverse OK" : "ERROR");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
    g_timing_offset = 0;
    g_invert_bits = 0;
}

// Simple ATR read - returns 1 if valid ATR received
uint8_t SIM_ReadATR_Simple(uint8_t *atr, uint8_t *len) {
    char msg[100];
    *len = 0;
    
    // Reset SIM
    HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_SET);
    
    SIM_SetInput();
    
    sprintf(msg, "ATR: ");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
    // Read ATR bytes
    for (int b = 0; b < 20; b++) {
        uint8_t byte = SIM_ReadOneByte();
        
        if (byte == 0xFF) break;  // Timeout
        
        atr[*len] = byte;
        (*len)++;
        
        sprintf(msg, "%02X ", byte);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    
    sprintf(msg, "\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
    // Valid if first byte is 0x3B or 0x3F
    return (*len > 0 && (atr[0] == 0x3B || atr[0] == 0x3F)) ? 1 : 0;
}

// Read ICCID from SIM (EF_ICCID = 2FE2)
void SIM_ReadICCID(char *iccid) {
    uint8_t response[32];
    uint8_t resp_len;
    
    strcpy(iccid, "ERROR");
    
    // SELECT MF (3F00)
    uint8_t sel_mf[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x3F, 0x00};
    SIM_SendAPDU(sel_mf, 7, response, &resp_len);
    if (resp_len >= 2 && response[0] == 0x9F) {
        SIM_GetResponse(response[1], response);
    }
    HAL_Delay(20);
    
    // SELECT EF_ICCID (2FE2) - directly under MF
    uint8_t sel_iccid[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x2F, 0xE2};
    SIM_SendAPDU(sel_iccid, 7, response, &resp_len);
    if (resp_len >= 2 && response[0] == 0x9F) {
        SIM_GetResponse(response[1], response);
    }
    HAL_Delay(20);
    
    // READ BINARY (10 bytes)
    uint8_t read_bin[] = {0xA0, 0xB0, 0x00, 0x00, 0x0A};
    SIM_SendAPDU(read_bin, 5, response, &resp_len);
    
    if (resp_len >= 10) {
        // ICCID is BCD encoded, swap nibbles
        int idx = 0;
        for (int i = 0; i < 10 && idx < 20; i++) {
            uint8_t lo = response[i] & 0x0F;
            uint8_t hi = (response[i] >> 4) & 0x0F;
            if (lo <= 9) iccid[idx++] = '0' + lo;
            if (hi <= 9) iccid[idx++] = '0' + hi;
        }
        iccid[idx] = '\0';
    }
}


// Initialize SIM card interface
uint8_t SIM_Init(void) {
    // Release I/O line
    HAL_GPIO_WritePin(SIM_IO_GPIO_Port, SIM_IO_Pin, GPIO_PIN_SET);
    
    // Set RST HIGH (inactive)
    HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_SET);
    
    return 1;  // SUCCESS
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
