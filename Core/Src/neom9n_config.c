/** 
 * @file    neom9n_config.c
 * @author  John Welgoss
 * @brief   NEO-M9N GPS Config File
 * 
 * USAGE:
 *  - First boot (GPS has factory defaults, SPI disabled):
 *      config_gps_seds_rocket_uart(&huart1, 5000);  // Configure via UART
 * 
 *  - Subsequent boots (SPI already enabled in flash):
 *      config_gps_seds_rocket_spi(&hspi1, 5000);    // Configure via SPI
 * 
 *  - Auto-detect (tries SPI first, falls back to UART):
 *      config_gps_seds_rocket(&hspi1, &huart1, 5000);
 * 
 * IMPORTANT: 
 *    What likely needs to happen:
 *     - First, configure board to use PA6 and PA7
 *       as huart2 tx rx for default UART configuration. 
 *     - Next, flash and run board, add usb printf to 
 *       confirm a successful/unsuccessful configuration.
 *     - Moving forward, revert PA6 and PA7 to MOSI and MISO
 *       and restore hspi1 handler. Any further config 
 *       changes shall change communication types (SPI forever).
 */

#include "neom9n_config.h"
#include "neom9n.h"
#include <string.h>

// Buffers for building UBX messages
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


/* -- SPI Transport Layer --------------------------- */

/**
 * @brief Send a UBX message over SPI
 */
 bool UBX_SendMessage_SPI(SPI_HandleTypeDef *hspi, uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len, uint32_t timeout) {
    uint16_t idx = 0;
    uint8_t ck_a, ck_b;
    
    // Build message header
    ubx_tx_buffer[idx++] = UBX_SYNC1;
    ubx_tx_buffer[idx++] = UBX_SYNC2;
    ubx_tx_buffer[idx++] = msg_class;
    ubx_tx_buffer[idx++] = msg_id;
    ubx_tx_buffer[idx++] = payload_len & 0xFF;
    ubx_tx_buffer[idx++] = (payload_len >> 8) & 0xFF;
    
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
    NEOGPS_CS_LOW();
    memset(&ubx_tx_buffer[idx], 0xFF, sizeof(ubx_tx_buffer) - idx);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, ubx_tx_buffer, ubx_rx_buffer, 
                                                        idx + 50, timeout);
    NEOGPS_CS_HIGH();
    
    return (status == HAL_OK);
}

/**
 * @brief Wait for ACK/NAK response from GPS via SPI
 */
bool UBX_WaitForAck_SPI(SPI_HandleTypeDef *hspi, uint8_t msg_class, uint8_t msg_id, uint32_t timeout) {
    uint8_t rx_byte;
    uint32_t start_tick = HAL_GetTick();
    uint8_t state = 0;
    uint8_t ack_class = 0, ack_id = 0, ack_msg_class = 0, ack_msg_id = 0;
    
    NEOGPS_CS_LOW();
    
    while ((HAL_GetTick() - start_tick) < timeout) {
        memset(ubx_tx_buffer, 0xFF, 1);
        if (HAL_SPI_TransmitReceive(hspi, ubx_tx_buffer, &rx_byte, 1, 10) != HAL_OK) {
            continue;
        }
        
        switch (state) {
            case 0: if (rx_byte == UBX_SYNC1) state = 1; break;
            case 1: state = (rx_byte == UBX_SYNC2) ? 2 : 0; break;
            case 2: ack_class = rx_byte; state = 3; break;
            case 3: ack_id = rx_byte; state = 4; break;
            case 4: state = (rx_byte == 0x02) ? 5 : 0; break;
            case 5: state = (rx_byte == 0x00) ? 6 : 0; break;
            case 6: ack_msg_class = rx_byte; state = 7; break;
            case 7:
                ack_msg_id = rx_byte;
                NEOGPS_CS_HIGH();
                if (ack_class == UBX_CLASS_ACK && ack_msg_class == msg_class && ack_msg_id == msg_id) {
                    return (ack_id == UBX_ACK_ACK);
                }
                state = 0;
                break;
        }
    }
    
    NEOGPS_CS_HIGH();
    return false;
}


/* -- UART Transport Layer -------------------------- */

/**
 * @brief Send a UBX message over UART
 */
bool UBX_SendMessage_UART(UART_HandleTypeDef *huart, uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len, uint32_t timeout) {
    uint16_t idx = 0;
    uint8_t ck_a, ck_b;
    
    // Build message
    ubx_tx_buffer[idx++] = UBX_SYNC1;
    ubx_tx_buffer[idx++] = UBX_SYNC2;
    ubx_tx_buffer[idx++] = msg_class;
    ubx_tx_buffer[idx++] = msg_id;
    ubx_tx_buffer[idx++] = payload_len & 0xFF;
    ubx_tx_buffer[idx++] = (payload_len >> 8) & 0xFF;
     
    if (payload_len > 0 && payload != NULL) {
        memcpy(&ubx_tx_buffer[idx], payload, payload_len);
        idx += payload_len;
    }
    
    UBX_CalculateChecksum(&ubx_tx_buffer[2], idx - 2, &ck_a, &ck_b);
    ubx_tx_buffer[idx++] = ck_a;
    ubx_tx_buffer[idx++] = ck_b;
    
    // Send over UART
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, ubx_tx_buffer, idx, timeout);
    
    return (status == HAL_OK);
}

/**
 * @brief Wait for ACK/NAK response from GPS via UART
 */
bool UBX_WaitForAck_UART(UART_HandleTypeDef *huart, uint8_t msg_class, uint8_t msg_id, uint32_t timeout) {
    uint8_t rx_byte;
    uint32_t start_tick = HAL_GetTick();
    uint8_t state = 0;
    uint8_t ack_class = 0, ack_id = 0, ack_msg_class = 0, ack_msg_id = 0;
    
    while ((HAL_GetTick() - start_tick) < timeout) {
        if (HAL_UART_Receive(huart, &rx_byte, 1, 10) != HAL_OK) {
            continue;
        }
        
        switch (state) {
            case 0: if (rx_byte == UBX_SYNC1) state = 1; break;
            case 1: state = (rx_byte == UBX_SYNC2) ? 2 : 0; break;
            case 2: ack_class = rx_byte; state = 3; break;
            case 3: ack_id = rx_byte; state = 4; break;
            case 4: state = (rx_byte == 0x02) ? 5 : 0; break;
            case 5: state = (rx_byte == 0x00) ? 6 : 0; break;
            case 6: ack_msg_class = rx_byte; state = 7; break;
            case 7:
                ack_msg_id = rx_byte;
                if (ack_class == UBX_CLASS_ACK && ack_msg_class == msg_class && ack_msg_id == msg_id) {
                    return (ack_id == UBX_ACK_ACK);
                }
                state = 0;
                break;
        }
    }
    
    return false;
}


/* -- Config helpers -------------------------------- */
 
bool UBX_SetConfigValue_SPI(SPI_HandleTypeDef *hspi, uint32_t key_id, uint32_t value, uint8_t value_size, uint8_t layers, uint32_t timeout) {
    uint8_t payload[64];
    uint16_t idx = 0;
    
    payload[idx++] = 0x01;      
    payload[idx++] = layers;
    payload[idx++] = 0x00;    
    payload[idx++] = 0x00;
    
    payload[idx++] = (key_id >> 0) & 0xFF;
    payload[idx++] = (key_id >> 8) & 0xFF;
    payload[idx++] = (key_id >> 16) & 0xFF;
    payload[idx++] = (key_id >> 24) & 0xFF;
    
    for (uint8_t i = 0; i < value_size; i++) {
        payload[idx++] = (value >> (i * 8)) & 0xFF;
    }
    
    if (!UBX_SendMessage_SPI(hspi, UBX_CLASS_CFG, UBX_CFG_VALSET, payload, idx, timeout)) {
        return false;
    }
    
    HAL_Delay(100);
    
    return UBX_WaitForAck_SPI(hspi, UBX_CLASS_CFG, UBX_CFG_VALSET, timeout);
}

bool UBX_SetConfigValue_UART(UART_HandleTypeDef *huart, uint32_t key_id, uint32_t value, uint8_t value_size, uint8_t layers, uint32_t timeout) {
    uint8_t payload[64];
    uint16_t idx = 0;
    
    payload[idx++] = 0x01;
    payload[idx++] = layers;
    payload[idx++] = 0x00;
    payload[idx++] = 0x00;
    
    payload[idx++] = (key_id >> 0) & 0xFF;
    payload[idx++] = (key_id >> 8) & 0xFF;
    payload[idx++] = (key_id >> 16) & 0xFF;
    payload[idx++] = (key_id >> 24) & 0xFF;
    
    for (uint8_t i = 0; i < value_size; i++) {
        payload[idx++] = (value >> (i * 8)) & 0xFF;
    }
    
    if (!UBX_SendMessage_UART(huart, UBX_CLASS_CFG, UBX_CFG_VALSET, payload, idx, timeout)) {
        return false;
    }
    
    HAL_Delay(100);
    
    return UBX_WaitForAck_UART(huart, UBX_CLASS_CFG, UBX_CFG_VALSET, timeout);
}


/* -- SPI configuration functions ------------------- */

bool set_dynamic_model_spi(SPI_HandleTypeDef *hspi, uint8_t model, uint32_t timeout) {
    return UBX_SetConfigValue_SPI(hspi, CFG_NAVSPG_DYNMODEL, model, 1, UBX_CFG_LAYER_ALL, timeout);
}

bool set_gps_update_rate_spi(SPI_HandleTypeDef *hspi, uint16_t rate_ms, uint32_t timeout) {
    if (!UBX_SetConfigValue_SPI(hspi, CFG_RATE_MEAS, rate_ms, 2, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);

    return UBX_SetConfigValue_SPI(hspi, CFG_RATE_NAV, 1, 2, UBX_CFG_LAYER_ALL, timeout);
}

bool enable_gps_spi_via_spi(SPI_HandleTypeDef *hspi, uint32_t timeout) {
    if (!UBX_SetConfigValue_SPI(hspi, CFG_SPI_ENABLED, 1, 1, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    
    return UBX_SetConfigValue_SPI(hspi, CFG_SPIOUTPROT_NMEA, 1, 1, UBX_CFG_LAYER_ALL, timeout);
}

bool config_gps_output_spi(SPI_HandleTypeDef *hspi, uint32_t timeout) {
    if (!UBX_SetConfigValue_SPI(hspi, CFG_MSGOUT_NMEA_ID_GGA_SPI, 1, 1, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    if (!UBX_SetConfigValue_SPI(hspi, CFG_MSGOUT_NMEA_ID_RMC_SPI, 1, 1, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    if (!UBX_SetConfigValue_SPI(hspi, CFG_MSGOUT_NMEA_ID_GLL_SPI, 0, 1, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    if (!UBX_SetConfigValue_SPI(hspi, CFG_MSGOUT_NMEA_ID_GSA_SPI, 0, 1, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    if (!UBX_SetConfigValue_SPI(hspi, CFG_MSGOUT_NMEA_ID_GSV_SPI, 0, 1, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    
    return UBX_SetConfigValue_SPI(hspi, CFG_MSGOUT_NMEA_ID_VTG_SPI, 0, 1, UBX_CFG_LAYER_ALL, timeout);
}

bool config_gps_seds_rocket_spi(SPI_HandleTypeDef *hspi, uint32_t timeout) {
    if (!enable_gps_spi_via_spi(hspi, timeout)) {
        return false;
    }
    HAL_Delay(200);
    if (!set_dynamic_model_spi(hspi, DYNMODEL_AIR4, timeout)) {
        return false;
    }
    HAL_Delay(200);
    if (!set_gps_update_rate_spi(hspi, 1000, timeout)) {
        return false;
    }
    HAL_Delay(200);
    if (!config_gps_output_spi(hspi, timeout)) {
        return false;
    }
    HAL_Delay(200);

    return true;
}


/* -- UART configuration functions ------------------ */

bool set_dynamic_model_uart(UART_HandleTypeDef *huart, uint8_t model, uint32_t timeout) {
    return UBX_SetConfigValue_UART(huart, CFG_NAVSPG_DYNMODEL, model, 1, UBX_CFG_LAYER_ALL, timeout);
}
 
bool set_gps_update_rate_uart(UART_HandleTypeDef *huart, uint16_t rate_ms, uint32_t timeout) {
    if (!UBX_SetConfigValue_UART(huart, CFG_RATE_MEAS, rate_ms, 2, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);

    return UBX_SetConfigValue_UART(huart, CFG_RATE_NAV, 1, 2, UBX_CFG_LAYER_ALL, timeout);
}
 
bool enable_gps_spi_via_uart(UART_HandleTypeDef *huart, uint32_t timeout) {
    if (!UBX_SetConfigValue_UART(huart, CFG_SPI_ENABLED, 1, 1, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);

    return UBX_SetConfigValue_UART(huart, CFG_SPIOUTPROT_NMEA, 1, 1, UBX_CFG_LAYER_ALL, timeout);
}
 
bool config_gps_output_uart(UART_HandleTypeDef *huart, uint32_t timeout) {
    if (!UBX_SetConfigValue_UART(huart, CFG_MSGOUT_NMEA_ID_GGA_SPI, 1, 1, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    if (!UBX_SetConfigValue_UART(huart, CFG_MSGOUT_NMEA_ID_RMC_SPI, 1, 1, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    if (!UBX_SetConfigValue_UART(huart, CFG_MSGOUT_NMEA_ID_GLL_SPI, 0, 1, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    if (!UBX_SetConfigValue_UART(huart, CFG_MSGOUT_NMEA_ID_GSA_SPI, 0, 1, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);
    if (!UBX_SetConfigValue_UART(huart, CFG_MSGOUT_NMEA_ID_GSV_SPI, 0, 1, UBX_CFG_LAYER_ALL, timeout)) {
        return false;
    }
    HAL_Delay(100);

    return UBX_SetConfigValue_UART(huart, CFG_MSGOUT_NMEA_ID_VTG_SPI, 0, 1, UBX_CFG_LAYER_ALL, timeout);
}
 
bool config_gps_seds_rocket_uart(UART_HandleTypeDef *huart, uint32_t timeout) {
    if (!enable_gps_spi_via_uart(huart, timeout)) {
        return false;
    }
    HAL_Delay(200);
    if (!set_dynamic_model_uart(huart, DYNMODEL_AIR4, timeout)) {
        return false;
    }
    HAL_Delay(200);
    if (!set_gps_update_rate_uart(huart, 1000, timeout)) {
        return false;
    }
    HAL_Delay(200);
    if (!config_gps_output_uart(huart, timeout)) {
        return false;
    }
    HAL_Delay(200);

    return true;
}


/* -- Auto-detect configuration --------------------- */

/**
 * @brief Auto-detect configuration - tries SPI first, falls back to UART
 * 
 * This is useful for:
 * - First boot: SPI disabled, must use UART
 * - Subsequent boots: SPI enabled, can use either
 */
bool config_gps_seds_rocket(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart, uint32_t timeout) {
    // Try SPI first (faster if already configured)
    if (config_gps_seds_rocket_spi(hspi, timeout)) {
        return true;
    }
     
    // SPI failed, try UART
    HAL_Delay(500);
    return config_gps_seds_rocket_uart(huart, timeout);
}