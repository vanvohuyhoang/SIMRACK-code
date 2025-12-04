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
    while (SIM_HAL_ReadIO() == 0 && timeout--) SIM_HAL_DelayUs(1);
    if (timeout == 0) return 0xFF;
    
    timeout = TIMEOUT_US;
    while (SIM_HAL_ReadIO() != 0 && timeout--) SIM_HAL_DelayUs(1);
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

