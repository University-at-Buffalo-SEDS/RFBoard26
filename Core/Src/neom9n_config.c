/** 
 * @file    neom9n_config.c
 * @author  John Welgoss
 * @brief   NEO-M9N GPS Config File
 */

#include "neom9n_config.h"
#include "neom9n.h"
#include "tx_api.h"
#include <string.h>

/* -- CONFIGURATION PRESETS ------------------------- */

/**
 * Safe rocket configuration - RAM only (not saved to flash)
 * If power cycles, reverts to factory defaults
 */
const gps_config_t GPS_CONFIG_ROCKET_SAFE = {
    .dynamic_model = DYNMODEL_AIR4,
    .update_rate_ms = 100,       // 10 Hz (100ms)
    .enable_gga = true,          // Has alt
    .enable_rmc = true,          // Has date
    .enable_gns = false,         // Redundant with GGA
    .layers = UBX_CFG_LAYER_RAM  // RAM only - SAFE
};

/**
 * Persistent rocket configuration - saved to flash
 * WARNING: If this fails mid-config, GPS might be misconfigured!
 * Only use after testing with SAFE config first.
 */
const gps_config_t GPS_CONFIG_ROCKET_PERSISTENT = {
    .dynamic_model = DYNMODEL_AIR4,
    .update_rate_ms = 1000,
    .enable_gga = true,
    .enable_rmc = true,
    .enable_gns = false,
    .layers = UBX_CFG_LAYER_RAM | UBX_CFG_LAYER_FLASH  // PERSISTENT
};

/**
 * Test configuration - portable mode, 1Hz
 */
const gps_config_t GPS_CONFIG_TEST = {
    .dynamic_model = DYNMODEL_PORTABLE,
    .update_rate_ms = 1000,
    .enable_gga = true,
    .enable_rmc = true,
    .enable_gns = false,
    .layers = UBX_CFG_LAYER_RAM
};

/* -- INTERNAL BUFFERS ------------------------------ */

static uint8_t ubx_tx_buffer[256];
static uint8_t ubx_rx_buffer[256];

/* -- LOW-LEVEL UBX FUNCTIONS ----------------------- */

/**
 * @brief Calculate UBX Fletcher checksum
 */
void ubx_calculate_checksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    for (uint16_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

/**
 * @brief Send UBX message over SPI
 */
bool ubx_send_message(SPI_HandleTypeDef *hspi, uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len, uint32_t timeout_ms) {
    uint16_t idx = 0;
    uint8_t ck_a, ck_b;
    
    // Build message header
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
    
    // Calculate checksum over CLASS, ID, LENGTH, PAYLOAD
    ubx_calculate_checksum(&ubx_tx_buffer[2], idx - 2, &ck_a, &ck_b);
    ubx_tx_buffer[idx++] = ck_a;
    ubx_tx_buffer[idx++] = ck_b;
    
    // Prepare for SPI transfer
    NEOGPS_CS_LOW();
    
    // Fill rest of buffer with 0xFF for SPI read
    memset(&ubx_tx_buffer[idx], 0xFF, sizeof(ubx_tx_buffer) - idx);
    
    // Send over SPI
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, ubx_tx_buffer, ubx_rx_buffer, idx + 50, timeout_ms);
    
    NEOGPS_CS_HIGH();
    
    return (status == HAL_OK);
}

/** 
 * @brief Wait for ACK/NAK response from GPS
 */
bool ubx_wait_for_ack(SPI_HandleTypeDef *hspi, uint8_t msg_class, uint8_t msg_id, uint32_t timeout_ms) {
    uint8_t rx_byte;
    uint32_t start_tick = HAL_GetTick();
    uint8_t state = 0;
    uint8_t ack_class = 0, ack_id = 0, ack_msg_class = 0, ack_msg_id = 0;
    
    NEOGPS_CS_LOW();
    
    // State machine to parse ACK message
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        // Read one byte
        memset(ubx_tx_buffer, 0xFF, 1);
        if (HAL_SPI_TransmitReceive(hspi, ubx_tx_buffer, &rx_byte, 1, 10) != HAL_OK) {
            continue;
        }
        
        switch (state) {
            case 0: // Looking for SYNC1
                if (rx_byte == UBX_SYNC1) state = 1;
                break;
                
            case 1: // Looking for SYNC2
                state = (rx_byte == UBX_SYNC2) ? 2 : 0;
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
                state = (rx_byte == 0x02) ? 5 : 0;
                break;
                
            case 5: // LENGTH MSB (should be 0 for ACK)
                state = (rx_byte == 0x00) ? 6 : 0;
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
                    return (ack_id == UBX_ACK_ACK);  // true if ACK, false if NAK
                }
                state = 0;
                break;
        }
    }
    
    NEOGPS_CS_HIGH();
    return false;  // Timeout
}

/**
 * @brief Set a single configuration value
 */
bool ubx_set_config(SPI_HandleTypeDef *hspi, uint32_t key_id, uint32_t value, uint8_t value_size, uint8_t layers, uint32_t timeout_ms) {
    uint8_t payload[64];
    uint16_t idx = 0;
    
    // CFG-VALSET header
    payload[idx++] = 0x01;      // Version
    payload[idx++] = layers;    // Layers to write
    payload[idx++] = 0x00;      // Reserved
    payload[idx++] = 0x00;      // Reserved
    
    // Key -> 4 bytes little-endian
    payload[idx++] = (key_id >> 0) & 0xFF;
    payload[idx++] = (key_id >> 8) & 0xFF;
    payload[idx++] = (key_id >> 16) & 0xFF;
    payload[idx++] = (key_id >> 24) & 0xFF;
    
    // Value -> 1, 2, 4, or 8 bytes little-endian
    for (uint8_t i = 0; i < value_size; i++) {
        payload[idx++] = (value >> (i * 8)) & 0xFF;
    }
    
    // Send message
    if (!ubx_send_message(hspi, UBX_CLASS_CFG, UBX_CFG_VALSET, payload, idx, timeout_ms)) {
        return false;
    }
    
    // Give GPS time to process
    tx_thread_sleep(100);
    
    // Wait for ACK
    return ubx_wait_for_ack(hspi, UBX_CLASS_CFG, UBX_CFG_VALSET, timeout_ms);
}

/* -- HIGH-LEVEL CONFIGURATION FUNCTIONS ------------ */

/**
 * @brief Test if GPS is responding on SPI
 */
bool gps_test_spi_connection(SPI_HandleTypeDef *hspi, uint32_t timeout_ms) {
    uint8_t test_tx[100];
    uint8_t test_rx[100];
    memset(test_tx, 0xFF, sizeof(test_tx));
    
    NEOGPS_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, test_tx, test_rx, 100, timeout_ms);
    NEOGPS_CS_HIGH();
    
    if (status != HAL_OK) {
        return false;
    }
    
    // Check if we got anything other than all 0xFF
    for (int i = 0; i < 100; i++) {
        if (test_rx[i] != 0xFF) {
            return true;  // GPS is responding
        }
    }
    
    return false;  // Only got 0xFF - GPS might not be transmitting
}

/**
 * @brief Set only dynamic model (minimum config for rockets)
 */
bool gps_set_dynamic_model_only(SPI_HandleTypeDef *hspi, gps_dynamic_model_t model, uint32_t timeout_ms) {
    // Only write to RAM - safest option
    return ubx_set_config(hspi, CFG_NAVSPG_DYNMODEL, (uint32_t)model, 1, UBX_CFG_LAYER_RAM, timeout_ms);
}

/**
 * @brief Configure GPS with custom settings
 */
bool gps_configure_custom(SPI_HandleTypeDef *hspi, const gps_config_t *config, uint32_t timeout_ms) {
    bool success = true;
    
    // Set dynamic model
    if (!ubx_set_config(hspi, CFG_NAVSPG_DYNMODEL, (uint32_t)config->dynamic_model, 1, config->layers, timeout_ms)) {
        success = false;
    }
    tx_thread_sleep(200);
    
    // Set measurement rate
    if (!ubx_set_config(hspi, CFG_RATE_MEAS, (uint32_t)config->update_rate_ms, 2, config->layers, timeout_ms)) {
        success = false;
    }
    tx_thread_sleep(200);
    
    // Set navigation rate (1 = every measurement)
    if (!ubx_set_config(hspi, CFG_RATE_NAV, 1, 2, config->layers, timeout_ms)) {
        success = false;
    }
    tx_thread_sleep(200);
    
    // Configure message outputs
    // Enable GGA
    if (!ubx_set_config(hspi, CFG_MSGOUT_NMEA_ID_GGA_SPI, config->enable_gga ? 1 : 0, 1, config->layers, timeout_ms)) {
        success = false;
    }
    tx_thread_sleep(100);
    
    // Enable RMC
    if (!ubx_set_config(hspi, CFG_MSGOUT_NMEA_ID_RMC_SPI, config->enable_rmc ? 1 : 0, 1, config->layers, timeout_ms)) {
        success = false;
    }
    tx_thread_sleep(100);
    
    // Enable GNS
    if (!ubx_set_config(hspi, CFG_MSGOUT_NMEA_ID_GNS_SPI, config->enable_gns ? 1 : 0, 1, config->layers, timeout_ms)) {
        success = false;
    }
    tx_thread_sleep(100);
    
    // Disable unnecessary messages
    ubx_set_config(hspi, CFG_MSGOUT_NMEA_ID_GLL_SPI, 0, 1, config->layers, timeout_ms); 
    tx_thread_sleep(100);
    ubx_set_config(hspi, CFG_MSGOUT_NMEA_ID_GSA_SPI, 0, 1, config->layers, timeout_ms); 
    tx_thread_sleep(100);
    ubx_set_config(hspi, CFG_MSGOUT_NMEA_ID_GSV_SPI, 0, 1, config->layers, timeout_ms);
    tx_thread_sleep(100);
    ubx_set_config(hspi, CFG_MSGOUT_NMEA_ID_VTG_SPI, 0, 1, config->layers, timeout_ms);
    tx_thread_sleep(100);
    
    return success;
}

/**
 * @brief Try to configure GPS with safe defaults
 */
bool gps_config(SPI_HandleTypeDef *hspi, uint32_t timeout_ms) {
    return gps_configure_custom(hspi, &GPS_CONFIG_ROCKET_SAFE, timeout_ms);  //use safe RAM-only config
}
 