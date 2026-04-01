/** 
 * @file    neom9n_config.h
 * @author  John Welgoss
 * @brief   NEO-M9N GPS Config Header File
 */

#ifndef NEOM9N_CONFIG_H
#define NEOM9N_CONFIG_H

/* -- Includes -------------------------------------------------------- */
#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>


/* -- Defines --------------------------------------------------------- */
// UBX Protocol Constants
#define UBX_SYNC1           0xB5
#define UBX_SYNC2           0x62
#define UBX_CLASS_CFG       0x06
#define UBX_CLASS_ACK       0x05
#define UBX_CFG_VALSET      0x8A
#define UBX_CFG_VALGET      0x8B
#define UBX_CFG_RST         0x04
#define UBX_ACK_ACK         0x01
#define UBX_ACK_NAK         0x00

// Configuration Layers
#define UBX_CFG_LAYER_RAM       0x01    
#define UBX_CFG_LAYER_BBR       0x02    
#define UBX_CFG_LAYER_FLASH     0x04    
#define UBX_CFG_LAYER_DEFAULT   0x08    

// Configuration Key IDs
// Dynamic Model
#define CFG_NAVSPG_DYNMODEL         0x20110021

// Update Rate 
#define CFG_RATE_MEAS               0x30210001 
#define CFG_RATE_NAV                0x30210002  

// SPI Interface 
#define CFG_SPI_ENABLED             0x10640006
#define CFG_SPIOUTPROT_NMEA         0x107a0002

// NMEA Message Output Rates on SPI 
#define CFG_MSGOUT_NMEA_ID_GGA_SPI  0x209100be
#define CFG_MSGOUT_NMEA_ID_RMC_SPI  0x209100af
#define CFG_MSGOUT_NMEA_ID_GLL_SPI  0x209100cd
#define CFG_MSGOUT_NMEA_ID_GSA_SPI  0x209100c3
#define CFG_MSGOUT_NMEA_ID_GSV_SPI  0x209100c8
#define CFG_MSGOUT_NMEA_ID_VTG_SPI  0x209100b4
#define CFG_MSGOUT_NMEA_ID_GNS_SPI  0x209100b9

// Dynamic model values 
typedef enum {
    DYNMODEL_PORTABLE   = 0,  // Default - max 12km alt, 310 m/s
    DYNMODEL_STATIONARY = 2,  // Stationary applications
    DYNMODEL_PEDESTRIAN = 3,  // Pedestrian
    DYNMODEL_AUTOMOTIVE = 4,  // Cars <4g
    DYNMODEL_SEA        = 5,  // Sea applications
    DYNMODEL_AIR1       = 6,  // Airborne <1g, max 50km alt
    DYNMODEL_AIR2       = 7,  // Airborne <2g, max 50km alt
    DYNMODEL_AIR4       = 8   // Airborne <4g, max 50km alt
} gps_dynamic_model_t;

// Config struct
typedef struct {
    gps_dynamic_model_t dynamic_model;  // AIR4
    uint16_t update_rate_ms;            // 1000 = 1Hz, 250 = 4Hz
    bool enable_gga;                    // GGA has: time, lat, lon, alt
    bool enable_rmc;                    // RMC has: time, lat, lon, date (no alt)
    bool enable_gns;                    // GNS has: time, lat, lon
    uint8_t layers;                     // Which layers to write to
} gps_config_t;

/* Preset configurations */
extern const gps_config_t GPS_CONFIG_ROCKET_SAFE;       
extern const gps_config_t GPS_CONFIG_ROCKET_PERSISTENT; 
extern const gps_config_t GPS_CONFIG_TEST;              // For ground testing


/* -- Function prototypes --------------------------------------------- */

/**
 * @brief Try to configure GPS - safe version
 * 
 * This function attempts to configure the GPS via SPI. If it fails,
 * it returns false but DOES NOT BRICK THE GPS. Factory defaults will
 * continue to work.
 * 
 * @param hspi SPI handle
 * @param timeout_ms Timeout in milliseconds
 * @return true if configuration succeeded, false otherwise
 * 
 * @note GPS will continue working with factory defaults if this fails
 */
bool gps_config(SPI_HandleTypeDef *hspi, uint32_t timeout_ms);

/**
 * @brief Configure GPS with custom settings
 * 
 * @param hspi SPI handle
 * @param config Configuration structure
 * @param timeout_ms Timeout in milliseconds
 * @return true if configuration succeeded
 */
bool gps_configure_custom(SPI_HandleTypeDef *hspi, const gps_config_t *config, uint32_t timeout_ms);

/**
 * @brief Set only the dynamic model (safest config change)
 * 
 * This is the MINIMUM configuration needed for rockets. Sets dynamic model
 * to AIR4 to enable high altitude/velocity operation.
 * 
 * @param hspi SPI handle
 * @param model Dynamic model (use DYNMODEL_AIR4 for rockets)
 * @param timeout_ms Timeout
 * @return true if successful
 */
bool gps_set_dynamic_model_only(SPI_HandleTypeDef *hspi, gps_dynamic_model_t model, uint32_t timeout_ms);

/**
 * @brief Check if GPS is responding on SPI
 * 
 * @param hspi SPI handle
 * @param timeout_ms Timeout
 * @return true if GPS responds
 */
bool gps_test_spi_connection(SPI_HandleTypeDef *hspi, uint32_t timeout_ms);

/* LOW-LEVEL UBX FUNCTIONS (not called directly) */

/**
 * @brief Send UBX configuration command
 * 
 * @param hspi SPI handle
 * @param key_id Configuration key ID
 * @param value Value to set
 * @param value_size Size of value (1, 2, 4, or 8 bytes)
 * @param layers Which layers to write to
 * @param timeout_ms Timeout
 * @return true if ACK received
 */
bool ubx_set_config(SPI_HandleTypeDef *hspi, uint32_t key_id, uint32_t value, uint8_t value_size, uint8_t layers, uint32_t timeout_ms);

/**
 * @brief Send raw UBX message
 * 
 * @param hspi SPI handle
 * @param msg_class UBX message class
 * @param msg_id UBX message ID
 * @param payload Payload data
 * @param payload_len Payload length
 * @param timeout_ms Timeout
 * @return true if sent successfully
 */
bool ubx_send_message(SPI_HandleTypeDef *hspi, uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len, uint32_t timeout_ms);

/**
 * @brief Wait for ACK/NAK response
 * 
 * @param hspi SPI handle
 * @param msg_class Expected message class
 * @param msg_id Expected message ID
 * @param timeout_ms Timeout
 * @return true if ACK received, false if NAK or timeout
 */
bool ubx_wait_for_ack(SPI_HandleTypeDef *hspi, uint8_t msg_class, uint8_t msg_id, uint32_t timeout_ms);

/**
 * @brief Calculate UBX checksum
 * 
 * @param data Data to checksum
 * @param len Length of data
 * @param ck_a Output: Checksum A
 * @param ck_b Output: Checksum B
 */
void ubx_calculate_checksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b);

#endif /* NEOM9N_CONFIG_H */
