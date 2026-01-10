/** 
 * @file    neom9n.h
 * @author  John Welgoss
 * @brief   NEO-M9N GPS Driver Header File
 */


#ifndef __CORE_INC_DRIVERS_NEOM9N_H 
#define __CORE_INC_DRIVERS_NEOM9N_H  

#include <stdint.h>  
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_spi.h"
#include "main.h"

// Globals
#define SPI_RX_BUFFER_SIZE 0x80    	// Size of SPI RX buffer in bytes
#define NMEA_PAYLOAD_RX_SIZE 0x20   // Size of NMEA payload RX buffer in bytes
#define MAX_NMEA_IGNORES 0x1F4      // Maximum number of NMEA sentences to ignore when searching for a valid one
#define NMEA_LATT_SIZE 0xB   		// Size of nmea lattitude word
#define NMEA_LONG_SIZE 0xC	 		// Size of nmea longitude word
#define NMEA_NS_SIZE 0x2   			// Size of nmea ns word
#define NMEA_EW_SIZE 0x2   			// Size of nmea ew word 
#define NMEA_STATUS_SIZE 0x2		// Size of nmea status word
#define NMEA_TMSTP_SIZE 0xA			// Size of nmea timestamp word 
#define NMEA_TALKERID_SIZE 0x6		// Size of nmea talkerID word

#define TAG(a,b,c) ((a << 16) | (b << 8) | (c))  //Packs size 3 str for state
uint8_t GLOBAL_HIGH_TX[SPI_RX_BUFFER_SIZE] = {[0 ... SPI_RX_BUFFER_SIZE-1] = 0xFF}; // Global TX buffer filled with 0xFF for SPI reads

// GPIO defs 
#define GPS_GPIO_PIN GPS_CS_Pin
#define GPS_GPIO_PORT GPS_CS_GPIO_Port

#define NEOGPS_CS_LOW() 											\
	HAL_GPIO_WritePin(GPS_GPIO_PORT, GPS_GPIO_PIN, GPIO_PIN_RESET)
#define NEOGPS_CS_HIGH() 											\
	HAL_GPIO_WritePin(GPS_GPIO_PORT, GPS_GPIO_PIN, GPIO_PIN_SET)

// Helper Functions

/**
 * @note Helper function to convert char to int (for cleaner code)
 * 
 * @brief Convert a character digit to its decimal integer value
 * @param c Character digit
 * @return Decimal integer value
 */
int char_to_int(uint8_t c) {
	return ((char)c) - '0';
}

// NEOM9N State Definition 
typedef enum {
    NEOM9N_GGA = TAG('G','G','A'),
    NEOM9N_GLL = TAG('G','L','L'),
    NEOM9N_GNS = TAG('G','N','S'),
    NEOM9N_RMC = TAG('R','M','C')
} NEOM9N_state_t;


// NEOM9N Configuration Structure
typedef struct {
    SPI_HandleTypeDef *hspi;
	float lat;
	float lon;
} NEOM9N_t;


// Function Prototypes
bool receive_nmea_payload(SPI_HandleTypeDef *hspi, uint8_t *tx, uint8_t *rx, uint16_t size, uint32_t max_wait);
void read_nmea_lat_and_long(NEOM9N_t *config, uint32_t max_wait);
void read_nmea_gga(NEOM9N_t *config, uint32_t max_wait);
void read_nmea_gll(NEOM9N_t *config, uint32_t max_wait);
void read_nmea_gns(NEOM9N_t *config, uint32_t max_wait);
void read_nmea_rmc(NEOM9N_t *config, uint32_t max_wait);
bool receive_nmea(NEOM9N_t *config, uint32_t max_wait, uint32_t max_ignores);



#endif /* __CORE_INC_DRIVERS_NEOM9N_H */