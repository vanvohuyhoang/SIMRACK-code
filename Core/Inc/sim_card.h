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
 * Authentication Forwarding (SIM Proxy)
 * For STM32 sitting between MODEM <-> SIM
 * ============================================================================ */

/**
 * @brief  GSM Authentication (RUN GSM ALGORITHM - 2G)
 *         Forwards RAND to SIM, returns SRES + Kc
 * 
 * @param  rand: 16-byte random challenge from modem
 * @param  sres: Output buffer for 4-byte Signed Response
 * @param  kc:   Output buffer for 8-byte Ciphering Key
 * @retval 1 = success, 0 = failure
 * 
 * Flow: MODEM sends RAND -> STM32 -> SIM -> SRES+Kc -> STM32 -> MODEM
 * APDU: A0 88 00 00 10 <RAND[16]>
 * Response: SRES[4] + Kc[8] + 9000
 */
uint8_t SIM_RunGSMAlgorithm(const uint8_t *rand, uint8_t *sres, uint8_t *kc);

/**
 * @brief  UMTS/3G Authentication (AUTHENTICATE - 3G/4G AKA)
 *         Forwards RAND+AUTN to USIM, returns full response for client
 * 
 * @param  rand: 16-byte random challenge from modem
 * @param  autn: 16-byte authentication token from modem
 * @param  response: Output buffer for full response (min 64 bytes)
 * @param  resp_len: Output - actual response length
 * @retval 1 = success, 0 = failure
 * 
 * APDU: 00 88 00 81 22 <RAND[16]> <AUTN[16]>
 * Success Response: DB <len> <RES> <CK[16]> <IK[16]> 9000
 * Sync Failure: DC <len> <AUTS[14]> 9000
 */
uint8_t SIM_Authenticate3G(const uint8_t *rand, const uint8_t *autn, 
                           uint8_t *response, uint8_t *resp_len);

/**
 * @brief  Generic Authentication - auto-detect 2G/3G
 *         Returns raw response bytes for direct forwarding to client
 * 
 * @param  rand: 16-byte RAND
 * @param  autn: 16-byte AUTN (NULL for 2G GSM auth)
 * @param  response: Output buffer for raw response (min 64 bytes)
 * @param  resp_len: Output - response length including SW
 * @retval 1 = success, 0 = failure
 */
uint8_t SIM_Authenticate(const uint8_t *rand, const uint8_t *autn,
                         uint8_t *response, uint8_t *resp_len);

/**
 * @brief  Select GSM directory (DF_GSM = 7F20)
 *         Must be called before authentication
 * @retval 1 = success, 0 = failure
 */
uint8_t SIM_SelectDFGSM(void);

/**
 * @brief  Select USIM ADF for 3G authentication
 * @retval 1 = success, 0 = failure  
 */
uint8_t SIM_SelectADFUSIM(void);

/**
 * @brief  Convert binary response to hex string for client transmission
 * @param  data: Binary data
 * @param  len: Data length
 * @param  hex_str: Output hex string (must be at least len*2+1 bytes)
 * 
 * Example: {0xFE, 0xF0, 0x90, 0x00} -> "FEF09000"
 */
void SIM_ResponseToHex(const uint8_t *data, uint8_t len, char *hex_str);

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

