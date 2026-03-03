/** 
 * @file    neom9n.h
 * @author  John Welgoss
 * @brief   NEO-M9N GPS Driver Header File
 */


#ifndef __CORE_INC_DRIVERS_NEOM9N_H 
#define __CORE_INC_DRIVERS_NEOM9N_H  

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_spi.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "main.h"

// Globals
#define NMEA_MAX_SENTENCE_LENGTH    0x52    // NMEA standard max length
#define NMEA_MAX_FIELD_LENGTH       0x10    // Max length of any single field
#define NMEA_MAX_IGNORES            0x1F4   // Max number of NMEA sentences to ignore when searching for a valid one
#define NMEA_MAX_WAIT               0x14    // Max timout for SPI transmition 
#define NEOM9N_PRECISION            0x4     // Decimal places for lat/lon


// GPIO defs 
#define GPS_GPIO_PIN GPS_CS_Pin
#define GPS_GPIO_PORT GPS_CS_GPIO_Port

#define NEOGPS_CS_LOW() HAL_GPIO_WritePin(GPS_GPIO_PORT, GPS_GPIO_PIN, GPIO_PIN_RESET)
#define NEOGPS_CS_HIGH() HAL_GPIO_WritePin(GPS_GPIO_PORT, GPS_GPIO_PIN, GPIO_PIN_SET)

// NEOM9N State Definition 
#define TAG(a,b,c) ((a << 16) | (b << 8) | (c))  //Packs size 3 str for state

typedef enum {
    NEOM9N_GGA = TAG('G','G','A'),
    NEOM9N_RMC = TAG('R','M','C')
} NEOM9N_state_t;


// NEOM9N status definition
typedef enum {
    NEOM9N_OK = 0,
    NEOM9N_TIMEOUT,
    NEOM9N_SPI_ERR,
    NEOM9N_PARSE_ERR,
    NEOM9N_BUFFER_OVERFLOW
} NEOM9N_status_t; 


// Updated GPS data packet structure
typedef struct {
    SPI_HandleTypeDef *hspi;    // SPI Handler
     
    // Position data
    float lat;              // Decimal degrees, positive = North -> (x)
    float lon;              // Decimal degrees, positive = East  -> (y)
    float altitude_msl;     // Alt above mean sea level (meters) -> (z)
     
    // Date (UTC)
    uint8_t day;
    uint8_t month;
    uint8_t year;

    // Time (UTC)
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint16_t milliseconds;
     
    // Status
    bool valid_fix;             // True if GPS has valid fix
    uint32_t last_update_tick;  // HAL tick of last successful update
     
    // Internal buffers for reduced stack usage
    uint8_t field_buffer[NMEA_MAX_FIELD_LENGTH]; 
} NEOM9N_t;

// Function prototypes
void gps_init(NEOM9N_t *packet, SPI_HandleTypeDef *hspi);
bool gps_has_fix(const NEOM9N_t *packet);
void pack_gps_data(const NEOM9N_t *packet, uint8_t *buffer);
uint64_t get_datetime_data(const NEOM9N_t *packet);
NEOM9N_status_t read_nmea_gga(NEOM9N_t *packet, uint32_t max_wait);
NEOM9N_status_t read_nmea_rmc(NEOM9N_t *packet, uint32_t max_wait);
NEOM9N_status_t receive_nmea(NEOM9N_t *packet, uint32_t max_wait, uint32_t max_ignores);


// Helper Functions
NEOM9N_status_t read_field(NEOM9N_t *packet, uint8_t *buffer, uint16_t max_len, uint32_t timeout);
void skip_field(NEOM9N_t *packet, uint32_t timeout);
float parse_coord(const uint8_t *field, uint8_t deg_digits);
float parse_float(const uint8_t *field);
void parse_date(const uint8_t *field, uint8_t *d, uint8_t *m, uint8_t *y);
void parse_time(const uint8_t *field, uint8_t *h, uint8_t *m, uint8_t *s, uint16_t *ms);


#endif /* __CORE_INC_DRIVERS_NEOM9N_H */