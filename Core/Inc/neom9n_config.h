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
#define UBX_CFG_VALSET      0x8A
#define UBX_CFG_VALGET      0x8B
#define UBX_CLASS_ACK       0x05
#define UBX_ACK_ACK         0x01
#define UBX_ACK_NAK         0x00

// Configuration layers
#define UBX_CFG_LAYER_RAM       0x1
#define UBX_CFG_LAYER_BBR       0x2
#define UBX_CFG_LAYER_FLASH     0x4
#define UBX_CFG_LAYER_ALL       0x7  // RAM + BBR + Flash

// Configuration Key IDs
#define CFG_NAVSPG_DYNMODEL         0x20110021  // Dynamic model
#define CFG_RATE_MEAS               0x30210001  // Measurement rate (ms)
#define CFG_RATE_NAV                0x30210002  // Navigation rate
#define CFG_SPI_ENABLED             0x10640006  // Enable SPI
#define CFG_SPIOUTPROT_NMEA         0x107a0002  // NMEA output on SPI
#define CFG_MSGOUT_NMEA_ID_GGA_SPI  0x209100be  // GGA output rate on SPI
#define CFG_MSGOUT_NMEA_ID_GLL_SPI  0x209100cd  // GLL output rate on SPI
#define CFG_MSGOUT_NMEA_ID_RMC_SPI  0x209100af  // RMC output rate on SPI
#define CFG_MSGOUT_NMEA_ID_GSA_SPI  0x209100c3  // GSA output rate on SPI
#define CFG_MSGOUT_NMEA_ID_GSV_SPI  0x209100c8  // GSV output rate on SPI
#define CFG_MSGOUT_NMEA_ID_VTG_SPI  0x209100b4  // VTG output rate on SPI

// Dynamic model values (E1 type)
#define DYNMODEL_PORTABLE       0x0
#define DYNMODEL_STATIONARY     0x2
#define DYNMODEL_PEDESTRIAN     0x3
#define DYNMODEL_AUTOMOTIVE     0x4
#define DYNMODEL_SEA            0x5
#define DYNMODEL_AIR1           0x6  // Airborne <1g
#define DYNMODEL_AIR2           0x7  // Airborne <2g 
#define DYNMODEL_AIR4           0x8  // Airborne <4g 


/* -- Function prototypes --------------------------------------------- */
bool NEOM9N_ConfigureForRocket(SPI_HandleTypeDef *hspi, uint32_t timeout);
bool NEOM9N_SetDynamicModel(SPI_HandleTypeDef *hspi, uint8_t model, uint32_t timeout);
bool NEOM9N_SetUpdateRate(SPI_HandleTypeDef *hspi, uint16_t rate_ms, uint32_t timeout);
bool NEOM9N_EnableSPI(SPI_HandleTypeDef *hspi, uint32_t timeout);
bool NEOM9N_ConfigureNMEAOutput(SPI_HandleTypeDef *hspi, uint32_t timeout);


/* -- Low-level UBX functions ----------------------------------------- */
bool UBX_SendMessage(SPI_HandleTypeDef *hspi, uint8_t msg_class, uint8_t msg_id, 
                     const uint8_t *payload, uint16_t payload_len, uint32_t timeout);
bool UBX_WaitForAck(SPI_HandleTypeDef *hspi, uint8_t msg_class, uint8_t msg_id, uint32_t timeout);
void UBX_CalculateChecksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b);

#endif /* NEOM9N_CONFIG_H */
