/** 
 * @file    neom9n_config.c
 * @author  John Welgoss
 * @brief   NEO-M9N GPS Config File
 * @note    Edit this file to configure:
 *              - Message types
 *              - Frequency
 *              - Communication Protocol
 */

#include "neom9n_config.h"
#include "neom9n.h"
#include <string.h>

// Buffer for building UBX messages
static uint8_t ubx_tx_buffer[256];
static uint8_t ubx_rx_buffer[256];

/**
 * @brief Calculate UBX Fletcher checksum
 */
void UBX_CalculateChecksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    for (uint16_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

/**
 * @brief Send a UBX message over SPI
 */
bool UBX_SendMessage(SPI_HandleTypeDef *hspi, uint8_t msg_class, uint8_t msg_id, 
                     const uint8_t *payload, uint16_t payload_len, uint32_t timeout) {
    uint16_t idx = 0;
    uint8_t ck_a, ck_b;
    
    // Message header
    ubx_tx_buffer[idx++] = UBX_SYNC1;
    ubx_tx_buffer[idx++] = UBX_SYNC2;
    ubx_tx_buffer[idx++] = msg_class;
    ubx_tx_buffer[idx++] = msg_id;
    ubx_tx_buffer[idx++] = payload_len & 0xFF;        // Length LSB
    ubx_tx_buffer[idx++] = (payload_len >> 8) & 0xFF; // Length MSB
    
    // Copy payload
    if (payload_len > 0 && payload != NULL) {
        memcpy(&ubx_tx_buffer[idx], payload, payload_len);
        idx += payload_len;
    }
    
    // Calculate checksum
    UBX_CalculateChecksum(&ubx_tx_buffer[2], idx - 2, &ck_a, &ck_b);
    ubx_tx_buffer[idx++] = ck_a;
    ubx_tx_buffer[idx++] = ck_b;
    
    // Send over SPI
    NEOGPS_CS_LOW();    // Begin transaction
    
    // Send the message, fill rest of buffer with 0xFF for reading response
    memset(&ubx_tx_buffer[idx], 0xFF, sizeof(ubx_tx_buffer) - idx);
    
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, ubx_tx_buffer, ubx_rx_buffer, 
                                                        idx + 50, timeout);
    
    NEOGPS_CS_HIGH();   // End transaction
    
    return (status == HAL_OK);
}

/**
 * @brief Wait for ACK/NAK response from GPS
 */
bool UBX_WaitForAck(SPI_HandleTypeDef *hspi, uint8_t msg_class, uint8_t msg_id, uint32_t timeout) {
    uint8_t rx_byte;
    uint32_t start_tick = HAL_GetTick();
    uint8_t state = 0;
    uint8_t ack_class = 0;
    uint8_t ack_id = 0;
    uint8_t ack_msg_class = 0;
    uint8_t ack_msg_id = 0;
    
    NEOGPS_CS_LOW();    // Begin transaction
    
    // State machine to find ACK message
    while ((HAL_GetTick() - start_tick) < timeout) {
        memset(ubx_tx_buffer, 0xFF, 1);  // Read one byte
        if (HAL_SPI_TransmitReceive(hspi, ubx_tx_buffer, &rx_byte, 1, 10) != HAL_OK) {
            continue;
        }
        
        switch (state) {
            case 0: // Looking for SYNC1
                if (rx_byte == UBX_SYNC1) state = 1;
                break;
            case 1: // Looking for SYNC2
                if (rx_byte == UBX_SYNC2) state = 2;
                else state = 0;
                break;
            case 2: // CLASS
                ack_class = rx_byte;
                state = 3;
                break;
            case 3: // ID
                ack_id = rx_byte;
                state = 4;
                break;
            case 4: // LENGTH LSB (should be 2 for ACK)
                if (rx_byte == 0x02) state = 5;
                else state = 0;
                break;
            case 5: // LENGTH MSB (should be 0 for ACK)
                if (rx_byte == 0x00) state = 6;
                else state = 0;
                break;
            case 6: // ACK message class
                ack_msg_class = rx_byte;
                state = 7;
                break;
            case 7: // ACK message ID
                ack_msg_id = rx_byte;
                NEOGPS_CS_HIGH();
                
                // Check if this is ACK for our message
                if (ack_class == UBX_CLASS_ACK && 
                    ack_msg_class == msg_class && 
                    ack_msg_id == msg_id) {
                    return (ack_id == UBX_ACK_ACK);  // True if ACK, false if NAK
                }
                state = 0;
                break;
        }
    }
    
    NEOGPS_CS_HIGH();   // End transaction
    return false;       // Timeout if switch-case is broken
}

/**
 * @brief Build and send a CFG-VALSET message with one key-value pair
 */
bool UBX_SetConfigValue(SPI_HandleTypeDef *hspi, uint32_t key_id, uint32_t value, 
                        uint8_t value_size, uint8_t layers, uint32_t timeout) {
    uint8_t payload[64];
    uint16_t idx = 0;
    
    // Header
    payload[idx++] = 0x01;      // Version
    payload[idx++] = layers;    // Layers (RAM + BBR + Flash)
    payload[idx++] = 0x00;      // Reserved
    payload[idx++] = 0x00;      // Reserved
    
    // Key-Value pair, Key: 4 bytes, little-endian
    payload[idx++] = (key_id >> 0) & 0xFF;
    payload[idx++] = (key_id >> 8) & 0xFF;
    payload[idx++] = (key_id >> 16) & 0xFF;
    payload[idx++] = (key_id >> 24) & 0xFF;
    
    // Value (1, 2, 4, or 8 bytes, little-endian)
    for (uint8_t i = 0; i < value_size; i++) {
        payload[idx++] = (value >> (i * 8)) & 0xFF;
    }
    
    // Send message
    if (!UBX_SendMessage(hspi, UBX_CLASS_CFG, UBX_CFG_VALSET, payload, idx, timeout)) {
        return false;
    }
    
    // Wait for ACK
    HAL_Delay(100);  // Give GPS time to process
    return UBX_WaitForAck(hspi, UBX_CLASS_CFG, UBX_CFG_VALSET, timeout);
}

/**
 * @brief Set dynamic model (AIR2 or AIR4 for rockets)
 */
bool set_dynamic_model(SPI_HandleTypeDef *hspi, uint8_t model, uint32_t timeout) {
    return UBX_SetConfigValue(hspi, CFG_NAVSPG_DYNMODEL, model, 1, 
                              UBX_CFG_LAYER_ALL, timeout);
}

/**
 * @brief Set update rate in milliseconds
 */
bool set_gps_update_rate(SPI_HandleTypeDef *hspi, uint16_t rate_ms, uint32_t timeout) {
    if (!UBX_SetConfigValue(hspi, CFG_RATE_MEAS, rate_ms, 2, 
                            UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    
    // Set navigation rate (1 = every measurement)
    HAL_Delay(100);
    return UBX_SetConfigValue(hspi, CFG_RATE_NAV, 1, 2, 
                              UBX_CFG_LAYER_ALL, timeout);
}

/**
 * @brief Enable SPI interface
 */
bool enable_gps_spi(SPI_HandleTypeDef *hspi, uint32_t timeout) {
    if (!UBX_SetConfigValue(hspi, CFG_SPI_ENABLED, 1, 1, 
                            UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    
    // Enable NMEA output protocol on SPI
    HAL_Delay(100);
    return UBX_SetConfigValue(hspi, CFG_SPIOUTPROT_NMEA, 1, 1, 
                              UBX_CFG_LAYER_ALL, timeout);
}

/**
 * @brief Configure NMEA message output (enable GGA, RMC | disable others)
 */
bool config_gps_output(SPI_HandleTypeDef *hspi, uint32_t timeout) {
    // Enable GGA (lat, lon, alt, no date, time)
    if (!UBX_SetConfigValue(hspi, CFG_MSGOUT_NMEA_ID_GGA_SPI, 1, 1, 
                            UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    
    // Enable RMC (lat, lon, no alt, date, time)
    if (!UBX_SetConfigValue(hspi, CFG_MSGOUT_NMEA_ID_RMC_SPI, 1, 1, 
                            UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    
    // Disable GLL
    if (!UBX_SetConfigValue(hspi, CFG_MSGOUT_NMEA_ID_GLL_SPI, 0, 1, 
                            UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    
    // Disable GSA 
    if (!UBX_SetConfigValue(hspi, CFG_MSGOUT_NMEA_ID_GSA_SPI, 0, 1, 
                            UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    
    // Disable GSV 
    if (!UBX_SetConfigValue(hspi, CFG_MSGOUT_NMEA_ID_GSV_SPI, 0, 1, 
                            UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    
    // Disable VTG 
    return UBX_SetConfigValue(hspi, CFG_MSGOUT_NMEA_ID_VTG_SPI, 0, 1, 
                              UBX_CFG_LAYER_ALL, timeout);
}

/**
 * @brief Complete configuration for rocket flight
 */
bool config_gps_seds_rocket(SPI_HandleTypeDef *hspi, uint32_t timeout) {
    // Enable SPI interface
    if (!enable_gps_spi(hspi, timeout)) {
        return false;
    }
    HAL_Delay(200);
    
    // Set dynamic model to AIR4
    if (!set_dynamic_model(hspi, DYNMODEL_AIR4, timeout)) {
        return false;
    }
    HAL_Delay(200);
    
    // Set update rate to 1 Hz (1000ms) (Change to 250ms for 4Hz if desired)
    if (!set_gps_update_rate(hspi, 1000, timeout)) {
        return false;
    }
    HAL_Delay(200);
    
    // Configure NMEA output
    if (!config_gps_output(hspi, timeout)) {
        return false;
    }
    HAL_Delay(200);
    
    return true;
}