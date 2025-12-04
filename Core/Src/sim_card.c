/**
 * @file    sim_card.c
 * @brief   ISO 7816-3 SIM Card Communication Driver Implementation
 * @details Platform-independent SIM card driver.
 *          Hardware-specific functions (SIM_HAL_*) must be implemented elsewhere.
 */

#include "sim_card.h"
#include "main.h"    // For inline HAL functions
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * Private Defines
 * ============================================================================ */

#define BIT_DELAY       SIM_BIT_DELAY_US
#define TIMEOUT_US      50000    // 50ms timeout for start bit detection

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static uint8_t SIM_GetResponse(uint8_t len, uint8_t *data);

/* ============================================================================
 * Inline macros for timing-critical operations (avoid function call overhead)
 * ============================================================================ */

#define SIM_SetOutput()     SIM_HAL_WriteIO(1)
#define SIM_SetInput()      SIM_HAL_WriteIO(1)
#define SIM_WriteBit(b)     SIM_HAL_WriteIO(b)
#define SIM_ReadBit()       SIM_HAL_ReadIO()

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

// Initialize SIM card interface
uint8_t SIM_Init(void) {
    // Release I/O line
    SIM_HAL_WriteIO(1);
    
    // Set RST HIGH (inactive)
    SIM_HAL_WriteRST(1);
    
    return 1;
}

// Reset SIM card
void SIM_Reset(void) {
    SIM_HAL_WriteRST(0);  // Assert reset
    SIM_HAL_DelayMs(20);
    SIM_HAL_WriteRST(1);  // Release reset
}

// Send a byte (ISO 7816-3 format)
void SIM_SendByte(uint8_t data) {
    uint8_t parity = 0;
    
    SIM_SetOutput();
    
    // Start bit (LOW)
    SIM_WriteBit(0);
    SIM_HAL_DelayUs(BIT_DELAY);
    
    // 8 data bits LSB first
    for (int i = 0; i < 8; i++) {
        uint8_t bit = (data >> i) & 1;
        SIM_WriteBit(bit);
        parity ^= bit;
        SIM_HAL_DelayUs(BIT_DELAY);
    }
    
    // Even parity bit
    SIM_WriteBit(parity);
    SIM_HAL_DelayUs(BIT_DELAY);
    
    // Guard time (2 stop bits)
    SIM_WriteBit(1);
    SIM_HAL_DelayUs(BIT_DELAY * 2);
}

// Receive a byte (waits for start bit)
uint8_t SIM_ReceiveByte(void) {
    uint8_t byte = 0;
    uint8_t bit;
    
    // Wait for idle (HIGH) then start bit (LOW)
    uint32_t timeout = TIMEOUT_US;
    while (SIM_HAL_ReadIO() == 0 && --timeout) SIM_HAL_DelayUs(1);
    if (timeout == 0) return 0xFF;
    
    timeout = TIMEOUT_US;
    while (SIM_HAL_ReadIO() != 0 && --timeout) SIM_HAL_DelayUs(1);
    if (timeout == 0) return 0xFF;
    
    // Wait 1.5 bit times to center on first data bit
    SIM_HAL_DelayUs(BIT_DELAY + (BIT_DELAY / 2));
    
    // Read 8 data bits (LSB first) - unrolled for consistent timing
    bit = SIM_HAL_ReadIO(); byte |= (bit << 0); SIM_HAL_DelayUs(BIT_DELAY);
    bit = SIM_HAL_ReadIO(); byte |= (bit << 1); SIM_HAL_DelayUs(BIT_DELAY);
    bit = SIM_HAL_ReadIO(); byte |= (bit << 2); SIM_HAL_DelayUs(BIT_DELAY);
    bit = SIM_HAL_ReadIO(); byte |= (bit << 3); SIM_HAL_DelayUs(BIT_DELAY);
    bit = SIM_HAL_ReadIO(); byte |= (bit << 4); SIM_HAL_DelayUs(BIT_DELAY);
    bit = SIM_HAL_ReadIO(); byte |= (bit << 5); SIM_HAL_DelayUs(BIT_DELAY);
    bit = SIM_HAL_ReadIO(); byte |= (bit << 6); SIM_HAL_DelayUs(BIT_DELAY);
    bit = SIM_HAL_ReadIO(); byte |= (bit << 7); SIM_HAL_DelayUs(BIT_DELAY);
    
    // Skip parity (already delayed after last bit)
    
    return byte;
}

// Receive multiple bytes
void SIM_ReceiveBytes(uint8_t *buf, uint8_t count) {
    for (int i = 0; i < count; i++) {
        buf[i] = SIM_ReceiveByte();
    }
}

// Read ATR (Answer To Reset)
uint8_t SIM_ReadATR(uint8_t *atr, uint8_t *len) {
    char msg[64];
    *len = 0;
    
    // Reset SIM
    SIM_Reset();
    SIM_SetInput();
    
    SIM_HAL_DebugPrint("ATR: ");
    
    // Read ATR bytes
    for (int b = 0; b < 20; b++) {
        uint8_t byte = SIM_ReceiveByte();
        
        if (byte == 0xFF) break;  // Timeout
        
        atr[*len] = byte;
        (*len)++;
        
        sprintf(msg, "%02X ", byte);
        SIM_HAL_DebugPrint(msg);
    }
    
    SIM_HAL_DebugPrint("\r\n");
    
    // Valid if first byte is 0x3B or 0x3F
    return (*len > 0 && (atr[0] == 0x3B || atr[0] == 0x3F)) ? 1 : 0;
}

// Send APDU and receive response
uint8_t SIM_SendAPDU(uint8_t *cmd, uint8_t cmd_len, uint8_t *response, uint8_t *resp_len) {
    char msg[80];
    *resp_len = 0;
    
    // Debug: show command
    SIM_HAL_DebugPrint("  CMD: ");
    for (int i = 0; i < cmd_len; i++) {
        sprintf(msg, "%02X ", cmd[i]);
        SIM_HAL_DebugPrint(msg);
    }
    SIM_HAL_DebugPrint("-> ");
    
    SIM_SetOutput();
    
    // Send command header (CLA INS P1 P2 P3)
    for (uint8_t i = 0; i < 5 && i < cmd_len; i++) {
        SIM_SendByte(cmd[i]);
    }
    
    SIM_SetInput();
    
    // Wait for procedure byte
    uint8_t proc_byte = SIM_ReceiveByte();
    sprintf(msg, "%02X ", proc_byte);
    SIM_HAL_DebugPrint(msg);
    
    if (proc_byte == 0xFF) {
        SIM_HAL_DebugPrint("(no response)\r\n");
        return 0;
    }
    
    // Handle NULL byte (0x60) - SIM needs more time
    while (proc_byte == 0x60) {
        SIM_HAL_DelayMs(10);
        proc_byte = SIM_ReceiveByte();
        sprintf(msg, "%02X ", proc_byte);
        SIM_HAL_DebugPrint(msg);
    }
    
    // If ACK (INS echoed)
    if (proc_byte == cmd[1]) {
        uint8_t ins = cmd[1];
        uint8_t p3 = cmd[4];
        uint8_t sw1 = 0, sw2 = 0;
        
        // READ commands: receive data
        if (ins == 0xB0 || ins == 0xB2 || ins == 0xC0) {
            // Read P3 data bytes + 2 status bytes
            uint8_t total = p3 + 2;
            SIM_ReceiveBytes(response, total);
            
            // Print data bytes
            for (int i = 0; i < p3; i++) {
                sprintf(msg, "%02X ", response[i]);
                SIM_HAL_DebugPrint(msg);
            }
            *resp_len = p3;
            
            sw1 = response[p3];
            sw2 = response[p3 + 1];
            sprintf(msg, "[%02X%02X]", sw1, sw2);
            SIM_HAL_DebugPrint(msg);
        } else {
            // WRITE commands with data
            if (cmd_len > 5) {
                SIM_SetOutput();
                for (uint8_t i = 5; i < cmd_len; i++) {
                    SIM_SendByte(cmd[i]);
                }
                SIM_SetInput();
            }
            
            // Read status words
            sw1 = SIM_ReceiveByte();
            sw2 = SIM_ReceiveByte();
            sprintf(msg, "[%02X%02X]", sw1, sw2);
            SIM_HAL_DebugPrint(msg);
            
            response[0] = sw1;
            response[1] = sw2;
            *resp_len = 2;
        }
        
        // If SW1=0x9F, there's data to read with GET RESPONSE
        if (sw1 == 0x9F) {
            uint8_t data_len = sw2;
            SIM_SetOutput();
            SIM_SendByte(0xA0);  // CLA
            SIM_SendByte(0xC0);  // INS = GET RESPONSE
            SIM_SendByte(0x00);  // P1
            SIM_SendByte(0x00);  // P2
            SIM_SendByte(data_len);  // Le
            SIM_SetInput();
            
            uint8_t ack = SIM_ReceiveByte();
            if (ack == 0xC0) {
                for (int i = 0; i < data_len; i++) {
                    response[i] = SIM_ReceiveByte();
                }
                *resp_len = data_len;
                // Read final status
                SIM_ReceiveByte();
                SIM_ReceiveByte();
            }
        }
    } else if ((proc_byte & 0xF0) == 0x90 || (proc_byte & 0xF0) == 0x60) {
        // Status word directly
        uint8_t sw2 = SIM_ReceiveByte();
        response[0] = proc_byte;
        response[1] = sw2;
        *resp_len = 2;
        sprintf(msg, "%02X ", sw2);
        SIM_HAL_DebugPrint(msg);
    }
    
    SIM_HAL_DebugPrint("\r\n");
    return (*resp_len > 0) ? 1 : 0;
}

// Simple GET RESPONSE command
static uint8_t SIM_GetResponse(uint8_t len, uint8_t *data) {
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
        SIM_HAL_DebugPrint(msg);
        
        for (int i = 0; i < len; i++) {
            data[i] = SIM_ReceiveByte();
            sprintf(msg, "%02X ", data[i]);
            SIM_HAL_DebugPrint(msg);
        }
        // Read SW1 SW2
        SIM_ReceiveByte();
        SIM_ReceiveByte();
        
        SIM_HAL_DebugPrint("\r\n");
        return 1;
    }
    return 0;
}

// Read IMSI
uint8_t SIM_ReadIMSI(char *imsi) {
    uint8_t response[64];
    uint8_t resp_len;
    
    strcpy(imsi, "ERROR");
    
    // Select MF (3F00)
    uint8_t select_mf[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x3F, 0x00};
    SIM_SendAPDU(select_mf, 7, response, &resp_len);
    if (resp_len >= 2 && response[0] == 0x9F) {
        SIM_GetResponse(response[1], response);
    }
    SIM_HAL_DelayMs(20);
    
    // Select DF_GSM (7F20)
    uint8_t select_gsm[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x7F, 0x20};
    SIM_SendAPDU(select_gsm, 7, response, &resp_len);
    if (resp_len >= 2 && response[0] == 0x9F) {
        SIM_GetResponse(response[1], response);
    }
    SIM_HAL_DelayMs(20);
    
    // Select EF_IMSI (6F07)
    uint8_t select_imsi[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x6F, 0x07};
    SIM_SendAPDU(select_imsi, 7, response, &resp_len);
    if (resp_len >= 2 && response[0] == 0x9F) {
        SIM_GetResponse(response[1], response);
    }
    SIM_HAL_DelayMs(20);
    
    // Read Binary (9 bytes)
    uint8_t read_bin[] = {0xA0, 0xB0, 0x00, 0x00, 0x09};
    SIM_SendAPDU(read_bin, 5, response, &resp_len);
    
    // Decode IMSI
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
        return 1;
    }
    return 0;
}

// Read ICCID
uint8_t SIM_ReadICCID(char *iccid) {
    uint8_t response[32];
    uint8_t resp_len;
    
    strcpy(iccid, "ERROR");
    
    // SELECT MF (3F00)
    uint8_t sel_mf[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x3F, 0x00};
    SIM_SendAPDU(sel_mf, 7, response, &resp_len);
    if (resp_len >= 2 && response[0] == 0x9F) {
        SIM_GetResponse(response[1], response);
    }
    SIM_HAL_DelayMs(20);
    
    // SELECT EF_ICCID (2FE2)
    uint8_t sel_iccid[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x2F, 0xE2};
    SIM_SendAPDU(sel_iccid, 7, response, &resp_len);
    if (resp_len >= 2 && response[0] == 0x9F) {
        SIM_GetResponse(response[1], response);
    }
    SIM_HAL_DelayMs(20);
    
    // READ BINARY (10 bytes)
    uint8_t read_bin[] = {0xA0, 0xB0, 0x00, 0x00, 0x0A};
    SIM_SendAPDU(read_bin, 5, response, &resp_len);
    
    // Decode ICCID (BCD, swap nibbles)
    if (resp_len >= 10) {
        int idx = 0;
        for (int i = 0; i < 10 && idx < 20; i++) {
            uint8_t lo = response[i] & 0x0F;
            uint8_t hi = (response[i] >> 4) & 0x0F;
            if (lo <= 9) iccid[idx++] = '0' + lo;
            if (hi <= 9) iccid[idx++] = '0' + hi;
        }
        iccid[idx] = '\0';
        return 1;
    }
    return 0;
}

/* ============================================================================
 * Authentication Forwarding (SIM Proxy)
 * ============================================================================ */

// Select DF_GSM (7F20) - required before authentication
uint8_t SIM_SelectDFGSM(void) {
    uint8_t response[32];
    uint8_t resp_len;
    
    // Select MF (3F00) first
    uint8_t sel_mf[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x3F, 0x00};
    SIM_SendAPDU(sel_mf, 7, response, &resp_len);
    if (resp_len >= 2 && response[0] == 0x9F) {
        SIM_GetResponse(response[1], response);
    }
    SIM_HAL_DelayMs(10);
    
    // Select DF_GSM (7F20)
    uint8_t sel_gsm[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x7F, 0x20};
    SIM_SendAPDU(sel_gsm, 7, response, &resp_len);
    if (resp_len >= 2 && response[0] == 0x9F) {
        SIM_GetResponse(response[1], response);
        return 1;
    }
    
    return (resp_len >= 2 && (response[0] == 0x90 || response[0] == 0x9F)) ? 1 : 0;
}

// GSM Authentication - RUN GSM ALGORITHM (INS 0x88)
// Input:  RAND (16 bytes) from modem
// Output: SRES (4 bytes) + Kc (8 bytes)
uint8_t SIM_RunGSMAlgorithm(const uint8_t *rand, uint8_t *sres, uint8_t *kc) {
    char msg[80];
    uint8_t response[32];
    uint8_t resp_len = 0;
    
    // Build RUN GSM ALGORITHM command
    // APDU: A0 88 00 00 10 <RAND[16]>
    uint8_t cmd[22];
    cmd[0] = 0xA0;  // CLA
    cmd[1] = 0x88;  // INS = RUN GSM ALGORITHM
    cmd[2] = 0x00;  // P1
    cmd[3] = 0x00;  // P2
    cmd[4] = 0x10;  // Lc = 16 bytes (RAND length)
    
    // Copy RAND
    for (int i = 0; i < 16; i++) {
        cmd[5 + i] = rand[i];
    }
    
    SIM_HAL_DebugPrint("  AUTH: ");
    for (int i = 0; i < 21; i++) {
        sprintf(msg, "%02X ", cmd[i]);
        SIM_HAL_DebugPrint(msg);
    }
    SIM_HAL_DebugPrint("-> ");
    
    // Send command header
    SIM_SetOutput();
    for (int i = 0; i < 5; i++) {
        SIM_SendByte(cmd[i]);
    }
    SIM_SetInput();
    
    // Wait for procedure byte
    uint8_t proc = SIM_ReceiveByte();
    sprintf(msg, "%02X ", proc);
    SIM_HAL_DebugPrint(msg);
    
    if (proc == 0xFF) {
        SIM_HAL_DebugPrint("(timeout)\r\n");
        return 0;
    }
    
    // If ACK (INS echoed = 0x88), send RAND data
    if (proc == 0x88) {
        SIM_SetOutput();
        for (int i = 0; i < 16; i++) {
            SIM_SendByte(rand[i]);
        }
        SIM_SetInput();
        
        // Read status word
        uint8_t sw1 = SIM_ReceiveByte();
        uint8_t sw2 = SIM_ReceiveByte();
        sprintf(msg, "[%02X%02X]", sw1, sw2);
        SIM_HAL_DebugPrint(msg);
        
        // SW1=0x9F means response data available
        if (sw1 == 0x9F && sw2 == 0x0C) {
            // GET RESPONSE to retrieve SRES + Kc (12 bytes)
            SIM_SetOutput();
            SIM_SendByte(0xA0);  // CLA
            SIM_SendByte(0xC0);  // INS = GET RESPONSE
            SIM_SendByte(0x00);  // P1
            SIM_SendByte(0x00);  // P2
            SIM_SendByte(0x0C);  // Le = 12 bytes
            SIM_SetInput();
            
            proc = SIM_ReceiveByte();
            if (proc == 0xC0) {
                // Read SRES (4 bytes)
                SIM_HAL_DebugPrint(" SRES:");
                for (int i = 0; i < 4; i++) {
                    sres[i] = SIM_ReceiveByte();
                    sprintf(msg, "%02X", sres[i]);
                    SIM_HAL_DebugPrint(msg);
                }
                
                // Read Kc (8 bytes)
                SIM_HAL_DebugPrint(" Kc:");
                for (int i = 0; i < 8; i++) {
                    kc[i] = SIM_ReceiveByte();
                    sprintf(msg, "%02X", kc[i]);
                    SIM_HAL_DebugPrint(msg);
                }
                
                // Read final status
                sw1 = SIM_ReceiveByte();
                sw2 = SIM_ReceiveByte();
                sprintf(msg, " [%02X%02X]\r\n", sw1, sw2);
                SIM_HAL_DebugPrint(msg);
                
                return (sw1 == 0x90 && sw2 == 0x00) ? 1 : 0;
            }
        }
        // Some SIMs return 9000 directly with data inline
        else if (sw1 == 0x90 && sw2 == 0x00) {
            SIM_HAL_DebugPrint(" (data already received)\r\n");
            return 1;
        }
    }
    
    SIM_HAL_DebugPrint("\r\n");
    return 0;
}

// Select ADF USIM for 3G authentication
uint8_t SIM_SelectADFUSIM(void) {
    uint8_t response[32];
    uint8_t resp_len;
    
    // SELECT by AID (USIM AID: A0000000871002...)
    // Most USIMs respond to partial AID selection
    uint8_t sel_usim[] = {0x00, 0xA4, 0x04, 0x04, 0x07, 
                          0xA0, 0x00, 0x00, 0x00, 0x87, 0x10, 0x02};
    
    SIM_SetOutput();
    for (int i = 0; i < 5; i++) SIM_SendByte(sel_usim[i]);
    SIM_SetInput();
    
    uint8_t proc = SIM_ReceiveByte();
    if (proc == 0xA4) {
        // Send AID data
        SIM_SetOutput();
        for (int i = 5; i < 12; i++) SIM_SendByte(sel_usim[i]);
        SIM_SetInput();
        
        uint8_t sw1 = SIM_ReceiveByte();
        uint8_t sw2 = SIM_ReceiveByte();
        
        // 61XX = success with data, 9000 = success
        if (sw1 == 0x61 || (sw1 == 0x90 && sw2 == 0x00)) {
            return 1;
        }
    }
    return 0;
}

// UMTS 3G Authentication (AUTHENTICATE command)
// Returns full response for forwarding to client
uint8_t SIM_Authenticate3G(const uint8_t *rand, const uint8_t *autn, 
                           uint8_t *response, uint8_t *resp_len) {
    char msg[80];
    *resp_len = 0;
    
    // AUTHENTICATE command for 3G context
    // CLA=00, INS=88, P1=00, P2=81 (3G context), Lc=22 (RAND+AUTN)
    SIM_HAL_DebugPrint("  3G-AUTH: 00 88 00 81 22 -> ");
    
    SIM_SetOutput();
    SIM_SendByte(0x00);  // CLA (USIM)
    SIM_SendByte(0x88);  // INS = AUTHENTICATE
    SIM_SendByte(0x00);  // P1
    SIM_SendByte(0x81);  // P2 = 3G context
    SIM_SendByte(0x22);  // Lc = 34 bytes (16 RAND + 2 len + 16 AUTN)
    SIM_SetInput();
    
    uint8_t proc = SIM_ReceiveByte();
    sprintf(msg, "%02X ", proc);
    SIM_HAL_DebugPrint(msg);
    
    if (proc == 0xFF) {
        SIM_HAL_DebugPrint("(timeout)\r\n");
        return 0;
    }
    
    // If ACK, send authentication data
    if (proc == 0x88) {
        SIM_SetOutput();
        
        // Send RAND length + RAND (TLV format for some SIMs)
        SIM_SendByte(0x10);  // RAND length = 16
        for (int i = 0; i < 16; i++) {
            SIM_SendByte(rand[i]);
        }
        
        // Send AUTN length + AUTN
        SIM_SendByte(0x10);  // AUTN length = 16
        for (int i = 0; i < 16; i++) {
            SIM_SendByte(autn[i]);
        }
        
        SIM_SetInput();
        
        // Read status
        uint8_t sw1 = SIM_ReceiveByte();
        uint8_t sw2 = SIM_ReceiveByte();
        sprintf(msg, "[%02X%02X]", sw1, sw2);
        SIM_HAL_DebugPrint(msg);
        
        // 61XX = response data available
        if (sw1 == 0x61) {
            uint8_t data_len = sw2;
            
            // GET RESPONSE
            SIM_SetOutput();
            SIM_SendByte(0x00);  // CLA
            SIM_SendByte(0xC0);  // INS = GET RESPONSE
            SIM_SendByte(0x00);  // P1
            SIM_SendByte(0x00);  // P2
            SIM_SendByte(data_len);  // Le
            SIM_SetInput();
            
            proc = SIM_ReceiveByte();
            if (proc == 0xC0) {
                SIM_HAL_DebugPrint(" DATA:");
                
                // Read all response data
                for (int i = 0; i < data_len; i++) {
                    response[i] = SIM_ReceiveByte();
                    sprintf(msg, "%02X", response[i]);
                    SIM_HAL_DebugPrint(msg);
                }
                
                // Read final SW
                sw1 = SIM_ReceiveByte();
                sw2 = SIM_ReceiveByte();
                
                // Append SW to response
                response[data_len] = sw1;
                response[data_len + 1] = sw2;
                *resp_len = data_len + 2;
                
                sprintf(msg, " [%02X%02X]\r\n", sw1, sw2);
                SIM_HAL_DebugPrint(msg);
                
                return (sw1 == 0x90 && sw2 == 0x00) ? 1 : 0;
            }
        }
        // 9000 = success (unlikely for 3G auth without GET RESPONSE)
        else if (sw1 == 0x90 && sw2 == 0x00) {
            response[0] = sw1;
            response[1] = sw2;
            *resp_len = 2;
            SIM_HAL_DebugPrint("\r\n");
            return 1;
        }
        // 9862 = authentication error
        else if (sw1 == 0x98 && sw2 == 0x62) {
            response[0] = sw1;
            response[1] = sw2;
            *resp_len = 2;
            SIM_HAL_DebugPrint(" (auth error)\r\n");
            return 0;
        }
    }
    
    SIM_HAL_DebugPrint("\r\n");
    return 0;
}

// Generic authentication - returns raw bytes for client forwarding
// If autn is NULL, uses 2G GSM auth; otherwise uses 3G UMTS auth
uint8_t SIM_Authenticate(const uint8_t *rand, const uint8_t *autn,
                         uint8_t *response, uint8_t *resp_len) {
    char msg[80];
    *resp_len = 0;
    
    if (autn == NULL) {
        // 2G GSM Authentication
        uint8_t sres[4], kc[8];
        
        if (SIM_RunGSMAlgorithm(rand, sres, kc)) {
            // Format: SRES(4) + Kc(8) + 9000
            for (int i = 0; i < 4; i++) response[i] = sres[i];
            for (int i = 0; i < 8; i++) response[4 + i] = kc[i];
            response[12] = 0x90;
            response[13] = 0x00;
            *resp_len = 14;
            return 1;
        }
        return 0;
    } else {
        // 3G UMTS Authentication
        return SIM_Authenticate3G(rand, autn, response, resp_len);
    }
}

// Helper: Convert response to hex string for client transmission
void SIM_ResponseToHex(const uint8_t *data, uint8_t len, char *hex_str) {
    for (int i = 0; i < len; i++) {
        sprintf(&hex_str[i * 2], "%02X", data[i]);
    }
    hex_str[len * 2] = '\0';
}
