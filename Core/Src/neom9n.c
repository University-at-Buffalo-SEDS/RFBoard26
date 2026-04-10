/** 
 * @file    neom9n.c
 * @author  John Welgoss
 * @brief   NEO-M9N GPS Driver File
 */

#include "neom9n.h"
#include "gps_time.h"
#include "main.h"
#include <stdio.h>

#ifndef NEOM9N_STDIO_DEBUG
#define NEOM9N_STDIO_DEBUG 0
#endif

static uint8_t GLOBAL_HIGH_TX[NMEA_MAX_SENTENCE_LENGTH] = {
    [0 ... NMEA_MAX_SENTENCE_LENGTH-1] = 0xFF
};  // Static TX buffer filled with 0xFF for SPI reads

/**
 * @brief Initialize GPS packet structure
 */
void gps_init(NEOM9N_t *packet, SPI_HandleTypeDef *hspi) {
    memset(packet, 0, sizeof(NEOM9N_t));
    packet->hspi = hspi;
}


/**
 * @brief Convert GPS datetime to milliseconds since UTC Epoch
 * 
 * Assumptions: 
 *  -year is yy (00–99) and full year is 2000 + yy
 *  -Gregorian calendar
 *  -UTC (no timezone offsets) 
 *  -Leap years handled correctly
 */
uint64_t get_datetime_data(const NEOM9N_t *packet) {
    return utc_to_epoch_ms(
        packet->day,
        packet->month,
        packet->year,
        packet->hours,
        packet->minutes,
        packet->seconds,
        packet->milliseconds
    );
}


/**
 * @brief Pack GPS position data as binary
 * 
 * Format: Pack as little-endian floats
 * Total: 12 bytes
 */
void pack_gps_data(const NEOM9N_t *packet, float *buffer) {
    buffer[0] = packet->lat;
    buffer[1] = packet->lon;
    buffer[2] = packet->altitude_msl;
}


/**
 * @brief Check if GPS has valid fix
 */
bool gps_has_fix(const NEOM9N_t *packet) {
    return packet->valid_fix;
}


/**
 * @brief Read one NMEA field 
 * @return 
 *      NEOM9N_OK -> if field read
 *      NEOM9N_PARSE_ERR -> if empty field
 *      NEOM9N_BUFFER_OVERFLOW -> if idx reaches max
 */
NEOM9N_status_t read_field(NEOM9N_t *packet, uint8_t *buffer, uint16_t max_len, uint32_t timeout) {
    uint16_t idx = 0;
    uint8_t byte;
    
    while (idx < max_len) {
        if (HAL_SPI_TransmitReceive(packet->hspi, GLOBAL_HIGH_TX, &byte, 1, timeout) != HAL_OK) {
            return NEOM9N_SPI_ERR;
        }
        
        if (byte == ',' || byte == '*') {
            buffer[idx] = '\0';
            if (idx == 0) {
                return NEOM9N_PARSE_ERR;
            }
            return NEOM9N_OK;
        }
        buffer[idx++] = byte;
    }
    
    return NEOM9N_BUFFER_OVERFLOW;
}


/**
 * @brief Skip to next comma or asterisk
 */
void skip_field(NEOM9N_t *packet, uint32_t timeout) {
    uint8_t byte;

    while (1) {
        HAL_SPI_TransmitReceive(packet->hspi, GLOBAL_HIGH_TX, &byte, 1, timeout);
        if (byte == ',' || byte == '*') {
            break;
        }
    }
}


/**
 * @brief Parse coordinate from NMEA format (ddmm.mmmmm or dddmm.mmmmm)
 * @param field Field containing coordinate string
 * @param deg_digits Number of degree digits (2 for lat, 3 for lon)
 * @return Coordinate in decimal degrees, or 0.0 on error
 */
float parse_coord(const uint8_t *field, uint8_t deg_digits) {
    // Parse degrees
    int32_t degrees = 0;
    for (uint8_t i = 0; i < deg_digits; i++) {
        if (!isdigit(field[i])) return 0.0f;
        degrees = degrees * 10 + (field[i] - '0');
    }
    
    // Parse minutes (mm.mmmmm format)
    float minutes = 0.0f;
    float divisor = 10.0f;
    bool past_decimal = false;
    
    for (uint8_t i = deg_digits; field[i] != '\0' && i < deg_digits + 9; i++) {
        if (field[i] == '.') {
            past_decimal = true;
            continue;
        }
        if (!isdigit(field[i])) {
            break;
        }
        if (!past_decimal) {
            minutes = minutes * 10 + (field[i] - '0');
        } 
        else {
            minutes += (field[i] - '0') / divisor;
            divisor *= 10.0f;
        }
    }
    return (float)degrees + (minutes / 60.0f);  // Convert to decimal degrees
}

/**
 * @brief Parse floating point number from field
 */
float parse_float(const uint8_t *field) {
    if (field[0] == '\0') {
        return 0.0f;
    }

    float result = 0.0f;
    int sign = 1;
    int idx = 0;
    
    if (field[0] == '-') {
        sign = -1;
        idx = 1;
    }
    
    // Integer part
    while (isdigit(field[idx])) {
        result = result * 10 + (field[idx] - '0');
        idx++;
    }
    
    // Decimal part
    if (field[idx] == '.') {
        idx++;
        float divisor = 10.0f;
        while (isdigit(field[idx])) {
            result += (field[idx] - '0') / divisor;
            divisor *= 10.0f;
            idx++;
        }
    }
    
    return result * sign;
}


/**
 * @brief Parse UTC date field (ddmmyy)
 */
void parse_date(const uint8_t *field, uint8_t *d, uint8_t *m, uint8_t *y) {
    if (strlen((char*)field) < 6) {
        *d = 0;
        *m = 0;
        *y = 0;
        return;
    }

    *d = (field[0] - '0') * 10 + (field[1] - '0');
    *m = (field[2] - '0') * 10 + (field[3] - '0');
    *y = (field[4] - '0') * 10 + (field[5] - '0');
}


/**
 * @brief Parse UTC time field (hhmmss.ss or hhmmss.sss)
 *
 * Handles fields where a leading zero may have been stripped,
 * PARSE: "22803.00" INSTEAD: of "022803.00".
 * Finds the decimal point (or end of str)
 * then read ss, mm, hh backwards from that anchor.
 */
void parse_time(const uint8_t *field, uint8_t *h, uint8_t *m, uint8_t *s, uint16_t *ms) {
    *h = *m = *s = 0;
    *ms = 0;

    // Find decimal point or end of integer part
    int dot_idx = -1;
    int len = (int)strlen((char*)field);

    for (int i = 0; i < len; i++) {
        if (field[i] == '.') {
            dot_idx = i;
            break;
        }
    }

    int int_end = (dot_idx >= 0) ? dot_idx : len;  // Index of first non-integer char

    // Need at least 6 integer digits (hhmmss), but tolerate 5 (hmmss) if leading zero stripped
    if (int_end < 5) return;

    // Parse backwards from int_end: ss at [-2:-1], mm at [-4:-3], hh at [-6:-5]
    *s = (field[int_end - 2] - '0') * 10 + (field[int_end - 1] - '0');
    *m = (field[int_end - 4] - '0') * 10 + (field[int_end - 3] - '0');

    if (int_end >= 6) {
        *h = (field[int_end - 6] - '0') * 10 + (field[int_end - 5] - '0');
    } else {
        *h = (field[0] - '0');  // 5 integer digits, leading zero was stripped, hour is single digit
    }

    // Parse fractional seconds -> millis
    if (dot_idx >= 0 && dot_idx + 1 < len) {
        uint16_t mult = 100;
        for (int i = dot_idx + 1; i < len && i < dot_idx + 4; i++) {
            if (isdigit(field[i])) {
                *ms += (field[i] - '0') * mult;
                mult /= 10;
            }
        }
    }
}

/**
 * @brief Parse GGA sentence: $GPGGA,time,lat,N/S,lon,E/W,quality,sats,hdop,alt,M,sep,...
 * 
 * EXTRACT: lat, lon, alt, time 
 * @note: GGA doesn't have date (we still update time in gps struct)
 */
NEOM9N_status_t read_nmea_gga(NEOM9N_t *packet, uint32_t max_wait) {
    NEOM9N_status_t status;
    
    // Time
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        parse_time(packet->field_buffer, &packet->hours, &packet->minutes, &packet->seconds, &packet->milliseconds);
    }
    
    // Latt
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status != NEOM9N_OK) {
        packet->valid_fix = false;
        return status;
    }
    float lat = parse_coord(packet->field_buffer, 2);  // ddmm.mmmm
    
    // N/S
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        packet->lat = (packet->field_buffer[0] == 'N') ? lat : -lat;
    }
    
    // Long
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status != NEOM9N_OK) {
        packet->valid_fix = false;
        return status;
    }
    float lon = parse_coord(packet->field_buffer, 3);  // dddmm.mmmm
    
    // E/W
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        packet->lon = (packet->field_buffer[0] == 'E') ? lon : -lon;
    }
    
    // Fix qual
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        uint8_t fix_quality = packet->field_buffer[0] - '0';
        packet->valid_fix = (fix_quality > 0);
    }
    
    // Number of satellites
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        uint8_t val = 0;
        for (int i = 0; packet->field_buffer[i] != '\0'; i++) {
            if (isdigit(packet->field_buffer[i])) {
                val = val * 10 + (packet->field_buffer[i] - '0');
            }
        }
        packet->num_satellites = val;
    }

    skip_field(packet, max_wait);    // HDOP
    
    // Alt
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        packet->altitude_msl = parse_float(packet->field_buffer);
    }
    
    return NEOM9N_OK;
}

/**
 * @brief Parse RMC sentence: $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,...
 * 
 * EXTRACT: date, time
 * @note: RMC doesn't have alt (gpsz)
 */
NEOM9N_status_t read_nmea_rmc(NEOM9N_t *packet, uint32_t max_wait) {
    NEOM9N_status_t status;
    
    // Time
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
#if NEOM9N_STDIO_DEBUG
        printf("RAW TIME FIELD: [%s] len=%d\r\n", packet->field_buffer, strlen((char*)packet->field_buffer));
#endif
        parse_time(packet->field_buffer, &packet->hours, &packet->minutes, &packet->seconds, &packet->milliseconds);
    }
    
    // Status
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        packet->valid_fix = (packet->field_buffer[0] == 'A');
    }
    
    // Latt
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status != NEOM9N_OK) return status;
    float lat = parse_coord(packet->field_buffer, 2);
    
    // N/S
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        packet->lat = (packet->field_buffer[0] == 'N') ? lat : -lat;
    }
    
    // Long
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status != NEOM9N_OK) return status;
    float lon = parse_coord(packet->field_buffer, 3);
    
    // E/W
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        packet->lon = (packet->field_buffer[0] == 'E') ? lon : -lon;
    }

    skip_field(packet, max_wait);   // Speed
    skip_field(packet, max_wait);   // Course

    // Date
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        parse_date(packet->field_buffer, &packet->day, &packet->month,&packet->year);
    }

    
    // Skip remaining fields
    // RMC doesn't provide alt, do not change from last entry

    return NEOM9N_OK;
}

/**
 * @brief Main function to receive and parse NMEA sentence
 */
NEOM9N_status_t receive_nmea(NEOM9N_t *packet, uint32_t max_wait, uint32_t max_ignores) {
    uint8_t byte;
    uint32_t ignores = 0;
    
    NEOGPS_CS_LOW();  // Begin transaction
    
    // Find sentence start at '$'
    while (ignores < max_ignores) {
        if (HAL_SPI_TransmitReceive(packet->hspi, GLOBAL_HIGH_TX, &byte, 1, max_wait) != HAL_OK) {
            NEOGPS_CS_HIGH();  // End transaction
            return NEOM9N_SPI_ERR;
        }
        if (byte == '$') break;
        ignores++;
    }
    
    if (byte != '$') {
        NEOGPS_CS_HIGH();  // End transaction
        return NEOM9N_TIMEOUT;
    }
    
    // Read talkerID + sentence type ("GPGGA" or "GPRMC")
    uint8_t header[6];
    if (HAL_SPI_TransmitReceive(packet->hspi, GLOBAL_HIGH_TX, header, 6, max_wait) != HAL_OK) {
        NEOGPS_CS_HIGH();  // End transaction
        return NEOM9N_SPI_ERR;
    }
    
    NEOM9N_state_t sentence_type = (header[2] << 16) | (header[3] << 8) | header[4];  // Extract sentence type (last 3 characters)
    HAL_SPI_TransmitReceive(packet->hspi, GLOBAL_HIGH_TX, &byte, 1, max_wait);  // Read comma after header
    
    // Parse based on sentence type
    NEOM9N_status_t result;
    switch (sentence_type) {
        case NEOM9N_GGA:
            result = read_nmea_gga(packet, max_wait);
            break;
        case NEOM9N_RMC:
            result = read_nmea_rmc(packet, max_wait);
            break;
        default:
            // Ignore sentence types we do not parse.
#if NEOM9N_STDIO_DEBUG
            printf("Not the desired sentence type - %s\r\n", header);
#endif
            result = NEOM9N_PARSE_ERR;
            break;
    }
    
    if (result == NEOM9N_OK) {
        packet->last_update_tick = HAL_GetTick();
    }
    
    NEOGPS_CS_HIGH();   // End transaction
    return result;
}
