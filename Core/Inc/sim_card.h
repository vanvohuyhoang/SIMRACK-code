/**
 * @file    sim_card.h
 * @brief   ISO 7816-3 SIM Card Communication Driver
 * @details Portable SIM card driver with hardware abstraction layer.
 *          Can be easily ported to different MCUs by implementing the HAL functions.
 * 
 * Usage:
 *   1. Implement the hardware abstraction functions (SIM_HAL_*)
 *   2. Call SIM_Init() once at startup
 *   3. Call SIM_ReadATR() after reset
 *   4. Use SIM_ReadIMSI() and SIM_ReadICCID() to read data
 */

#ifndef SIM_CARD_H
#define SIM_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================================
 * Configuration
 * ============================================================================ */

// Timing parameters (for ~3.6MHz SIM clock)
#define SIM_BIT_DELAY_US    95    // Bit period in microseconds (ETU)

// Buffer sizes
#define SIM_ATR_MAX_LEN     33    // Maximum ATR length
#define SIM_IMSI_MAX_LEN    16    // Maximum IMSI string length
#define SIM_ICCID_MAX_LEN   21    // Maximum ICCID string length

/* ============================================================================
 * Hardware Abstraction Layer (HAL) - MUST be implemented per platform
 * ============================================================================ */

/*
 * Hardware Abstraction Layer (HAL) Functions
 * 
 * These functions MUST be implemented per platform.
 * For STM32: inline implementations are in main.h
 * 
 * Required functions:
 *   void SIM_HAL_DelayUs(uint32_t us)   - Microsecond delay (MUST be fast/inline)
 *   void SIM_HAL_DelayMs(uint32_t ms)   - Millisecond delay  
 *   void SIM_HAL_WriteIO(uint8_t level) - Write I/O pin (MUST be fast/inline)
 *   uint8_t SIM_HAL_ReadIO(void)        - Read I/O pin (MUST be fast/inline)
 *   void SIM_HAL_WriteRST(uint8_t level)- Control RST pin
 *   void SIM_HAL_DebugPrint(const char*)- Debug output (optional)
 */

/* ============================================================================
 * SIM Card API
 * ============================================================================ */

/**
 * @brief  Initialize SIM card interface
 * @retval 1 = success, 0 = failure
 */
uint8_t SIM_Init(void);

/**
 * @brief  Read Answer To Reset (ATR)
 * @param  atr: Buffer to store ATR bytes
 * @param  len: Pointer to store ATR length
 * @retval 1 = valid ATR, 0 = failed
 */
uint8_t SIM_ReadATR(uint8_t *atr, uint8_t *len);

/**
 * @brief  Read IMSI from SIM card
 * @param  imsi: Buffer to store IMSI string (min 16 bytes)
 * @retval 1 = success, 0 = failure
 */
uint8_t SIM_ReadIMSI(char *imsi);

/**
 * @brief  Read ICCID from SIM card  
 * @param  iccid: Buffer to store ICCID string (min 21 bytes)
 * @retval 1 = success, 0 = failure
 */
uint8_t SIM_ReadICCID(char *iccid);

/**
 * @brief  Send APDU command and receive response
 * @param  cmd: APDU command bytes (header + optional data)
 * @param  cmd_len: Command length
 * @param  response: Buffer for response data
 * @param  resp_len: Pointer to store response length
 * @retval 1 = success, 0 = failure
 */
uint8_t SIM_SendAPDU(uint8_t *cmd, uint8_t cmd_len, uint8_t *response, uint8_t *resp_len);

/**
 * @brief  Reset SIM card (cold reset)
 */
void SIM_Reset(void);

/* ============================================================================
 * Low-level functions (for advanced use)
 * ============================================================================ */

/**
 * @brief  Send a byte (ISO 7816-3: start bit, 8 data LSB first, even parity, stop)
 */
void SIM_SendByte(uint8_t data);

/**
 * @brief  Receive a byte (waits for start bit)
 * @retval Received byte, or 0xFF on timeout
 */
uint8_t SIM_ReceiveByte(void);

/**
 * @brief  Receive multiple bytes
 * @param  buf: Buffer for received bytes
 * @param  count: Number of bytes to receive
 */
void SIM_ReceiveBytes(uint8_t *buf, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* SIM_CARD_H */

