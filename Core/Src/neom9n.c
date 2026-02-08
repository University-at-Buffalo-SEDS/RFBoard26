/** 
 * @file    neom9n.c
 * @author  John Welgoss
 * @brief   NEO-M9N GPS Driver File
 */

#include "neom9n.h"
#include "main.h"
#include <stdio.h>

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
 * @brief Convert GPS time to seconds since midnight UTC
 */
double time_to_seconds(const NEOM9N_t *packet) {
    return (double)packet->hours * 3600.0 + 
           (double)packet->minutes * 60.0 + 
           (double)packet->seconds + 
           (double)packet->milliseconds / 1000.0;
}


/**
 * @brief Pack GPS position data as binary
 * 
 * Format: Pack as little-endian floats
 * Total: 12 bytes
 */
void pack_gps_data(const NEOM9N_t *packet, uint8_t *buffer) {
    float *float_ptr = (float *)buffer;
    float_ptr[0] = packet->lat;
    float_ptr[1] = packet->lon;
    float_ptr[2] = packet->altitude_msl;
}


/**
 * @brief Pack GPS time as u64
 * 
 * Format: Pack as little-endian double
 * Total: 8 bytes
 */
void pack_time_data(const NEOM9N_t *packet, uint64_t *buffer) {
    
    *buffer = (uint64_t)(time_to_seconds(packet) * 1000.0);  // Convert to ms and pack as u64
    return;
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
 * @brief Parse UTC time field (hhmmss.sss)
 */
void parse_time(const uint8_t *field, uint8_t *h, uint8_t *m, uint8_t *s, uint16_t *ms) {
    if (strlen((char*)field) < 6) {
        *h = 0;
        *m = 0;
        *s = 0;
        *ms = 0;
        return;
    }
    
    *h = (field[0] - '0') * 10 + (field[1] - '0');
    *m = (field[2] - '0') * 10 + (field[3] - '0');
    *s = (field[4] - '0') * 10 + (field[5] - '0');
    *ms = 0;

    if (field[6] == '.' && strlen((char*)field) > 7) {
        uint16_t mult = 100;
        for (int i = 7; field[i] != '\0' && i < 10; i++) {
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
 * Extract: lat, lon, alt, time 
 */
NEOM9N_status_t read_nmea_gga(NEOM9N_t *packet, uint32_t max_wait) {
    NEOM9N_status_t status;
    
    // Time
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        parse_time(packet->field_buffer, &packet->hours, &packet->minutes, 
                  &packet->seconds, &packet->milliseconds);
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
    
    skip_field(packet, max_wait);    // Number of satalites
    skip_field(packet, max_wait);    // HDOP
    
    // Alt
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        packet->altitude_msl = parse_float(packet->field_buffer);
    }
    
    return NEOM9N_OK;
}

/**
 * @brief Parse GLL sentence: $GPGLL,lat,N/S,lon,E/W,time,status,mode
 * 
 * Extract: lat, lon, time
 * @note: GLL doesn't have alt (gpsz)
 */
NEOM9N_status_t read_nmea_gll(NEOM9N_t *packet, uint32_t max_wait) {
    NEOM9N_status_t status;
    
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
    
    // Time
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        parse_time(packet->field_buffer, &packet->hours, &packet->minutes,
                  &packet->seconds, &packet->milliseconds);
    }
    
    // Status
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        packet->valid_fix = (packet->field_buffer[0] == 'A');
    }
    
    //GLL doesn't provide alt, do not change from last entry
    
    return NEOM9N_OK;
}

/**
 * @brief Parse RMC sentence: $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,...
 * 
 * Extract: lat, lon, time
 * @note: RMC doesn't have alt (gpsz)
 */
NEOM9N_status_t read_nmea_rmc(NEOM9N_t *packet, uint32_t max_wait) {
    NEOM9N_status_t status;
    
    // Time
    status = read_field(packet, packet->field_buffer, sizeof(packet->field_buffer), max_wait);
    if (status == NEOM9N_OK) {
        parse_time(packet->field_buffer, &packet->hours, &packet->minutes,
                  &packet->seconds, &packet->milliseconds);
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
    
    // Skip remaining fields
    // RMC doesn't provide alt, do not change from last entry

    return NEOM9N_OK;
}

/**
 * @brief Parse GNS sentence: $GPGNS,time,lat,N/S,lon,E/W,mode,numSV,HDOP,alt,sep,...
 * 
 * Extract: lat, lon, alt, time
 * GNS is same as GGA for our current use case
 */
NEOM9N_status_t read_nmea_gns(NEOM9N_t *packet, uint32_t max_wait) {
    return read_nmea_gga(packet, max_wait);
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
    
    // Read talkerID + sentence type ("GPGGA", "GNGLL", ...)
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
        case NEOM9N_GLL:
            result = read_nmea_gll(packet, max_wait);
            break;
        case NEOM9N_RMC:
            result = read_nmea_rmc(packet, max_wait);
            break;
        case NEOM9N_GNS:
            result = read_nmea_gns(packet, max_wait);
            break;
        default:
            NEOGPS_CS_HIGH();  // End transaction
            return NEOM9N_PARSE_ERR;
    }
    
    if (result == NEOM9N_OK) {
        packet->last_update_tick = HAL_GetTick();
    }
    
    NEOGPS_CS_HIGH();   // End transaction
    return result;
}