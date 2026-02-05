/** 
 * @file    neom9n.c
 * @author  John Welgoss
 * @brief   NEO-M9N GPS Driver File
 */

#include "neom9n.h"
#include "telemetry.h"
#include "main.h"
#include <stdio.h>

uint8_t GLOBAL_HIGH_TX[SPI_RX_BUFFER_SIZE] = {[0 ... SPI_RX_BUFFER_SIZE-1] = 0xFF}; // Global TX buffer filled with 0xFF for SPI reads

/**
 * @note Helper function to convert char to int
 * 
 * @brief Convert a character digit to its decimal integer value
 * @param c Character digit
 * @return Decimal integer value
 */
int char_to_uint(uint8_t c) {
	return ((char)c) - '0';
}


/**
 * @note Helper function to convert float to str
 * 
 * @brief Convert a float into a char[] of its ascii chars
 * @param value float to be converted
 * @param buffer buffer to store result
 * @param precision location of decimal point 
 */
void float_to_str(float value, char* buffer, int precision) {
	int int_part = (int)value;
    
    int multiplier = 1;
    for (int i = 0; i < precision; i++) {
        multiplier *= 10;
    }   // Calculate 10^precision
    
    int frac_part = (int)((value - int_part) * multiplier);
    if (frac_part < 0) {
		frac_part = -frac_part;
	}
    
    // Build format string dynamically (e.g., "%d.%04d" for precision=4)
    char format[16];
    sprintf(format, "%%d.%%0%dd", precision);
    
    sprintf(buffer, format, int_part, frac_part);
}


/**
 * @brief Parse Coord from NMEA payload
 * @param payload Pointer to NMEA payload containing latitude
 * @param degDigits for n digits
 * @return Coord in decimal degrees
 * 
 * Format: ddmm.mmmmm
 */
float parseCoord(uint8_t *payload, int degDigits) {
    int deg = 0;
    for (int i = 0; i < degDigits; i++) {
		if (!isdigit(payload[i])) {
			return NEOM9N_PARSE_ERR;
		}

        deg = deg * 10 + char_to_uint(payload[i]);
    }

    float min = 0;
    float pos = 10;
    for (int i = 0; i < 8; i++) {
        if (i != 2) {
			if (!isdigit(payload[degDigits + i])) {
				return NEOM9N_PARSE_ERR;
			}

            min += pos * char_to_uint(payload[degDigits + i]);
            pos /= 10;
        }
    }
    return (float)deg + min / 60;
}


/**
 * @brief Receive a section of NMEA payload until a comma or size bytes
 * @param hspi Pointer to SPI handle
 * @param tx Pointer to TX buffer
 * @param rx Pointer to RX buffer
 * @param size Maximum number of bytes to read
 * @param max_wait Maximum wait time for SPI operations
 * @return NEOM9N_OK if data was read, NEOM9N_PARSE_ERR if a comma was encountered first
 * 
 * Reads bytes into rx until a comma is encountered or size bytes have been read.
 * The first byte is checked to see if it's a comma; if so, no further bytes are read.
 */
NEOM9N_status_t receive_nmea_payload(SPI_HandleTypeDef *hspi, uint8_t *tx, uint8_t *rx, uint16_t size, uint32_t max_wait) { 
	//stop at ',' or size bytes
	HAL_SPI_TransmitReceive(hspi, tx, rx, 1, max_wait);  //read first byte 
	
	if (rx[0] == ',') {
		return NEOM9N_PARSE_ERR;   //if first byte is comma, stop reading
	}

	HAL_SPI_TransmitReceive(hspi, tx, rx+1, size-1, max_wait); 	//otherwise continue to read the rest into rx

	return NEOM9N_OK;
}


/**
 * @brief Read latitude and longitude from NMEA payload
 * @param packet Pointer to NeoGPS packet structure
 * @param max_wait Maximum wait time for SPI operations
 * 
 * Reads latitude and longitude from the NMEA payload and updates the packet struct.
 */
void read_nmea_lat_and_long(NEOM9N_t *packet, uint32_t max_wait) {
	uint8_t rx[NMEA_PAYLOAD_RX_SIZE]; //buffer for receiving nmea payload sections

	//receive the latitude(ddmm.mmmmm)+ comma after
	float recv_lat = packet->lat;
	if (receive_nmea_payload(packet->hspi, GLOBAL_HIGH_TX, rx, NMEA_LATT_SIZE, max_wait) == NEOM9N_OK) {
		recv_lat = parseCoord(rx,NMEA_LATT_SIZE);
	}

	//receive the NS indicator and comma after
	uint8_t ns_indicator;
	if (receive_nmea_payload(packet->hspi, GLOBAL_HIGH_TX, &ns_indicator, NMEA_NS_SIZE, max_wait) == NEOM9N_OK) {
		if (ns_indicator == 'N') {
			packet->lat = recv_lat;
		} else {
			packet->lat = -recv_lat;
		}
		
	}

	//receive the longitude(ddmm.mmmmm)+ comma after
	float recv_long = packet->lon;
	if (receive_nmea_payload(packet->hspi, GLOBAL_HIGH_TX, rx, NMEA_LONG_SIZE, max_wait) == NEOM9N_OK) {
		recv_long = parseCoord(rx,NMEA_LONG_SIZE);
	}

	//receive the EW indicator and comma after
	uint8_t ew_indicator;
	if (receive_nmea_payload(packet->hspi, GLOBAL_HIGH_TX, &ew_indicator, NMEA_EW_SIZE, max_wait) == NEOM9N_OK) {
		if (ew_indicator == 'E') {
			packet->lon = recv_long;
		} else {
			packet->lon = -recv_long;
		}
		
	}
}


/**
 * @brief Read NMEA GGA sentence, and NMEA GNS sentence
 * @param packet Pointer to NeoGPS packet structure
 * @param max_wait Maximum wait time for SPI operations
 * 
 * Reads the GGA NMEA sentence and extracts latitude and longitude.
 * @format: time,lat,NS,lon,EW,fixQuality,numSV,HDOP,alt,sep,diffAge,diffStation,navStatus*cs\r\n
 */
void read_nmea_gga(NEOM9N_t *packet, uint32_t max_wait) {
	uint8_t rx[NMEA_PAYLOAD_RX_SIZE];

	//receive the time ( hhmmss.ss) + comma after
	receive_nmea_payload(packet->hspi, GLOBAL_HIGH_TX, rx, NMEA_TMSTP_SIZE, max_wait);

	read_nmea_lat_and_long(packet, max_wait);
}


/**
 * @brief Read NMEA GLL sentence
 * @param packet Pointer to NeoGPS packet structure
 * @param max_wait Maximum wait time for SPI operations
 * 
 * Reads the GLL NMEA sentence and extracts latitude and longitude.
 * @format: lat,NS,lon,EW,time,status,posMode*cs\r\n
 */
void read_nmea_gll(NEOM9N_t *packet, uint32_t max_wait) {
	read_nmea_lat_and_long(packet, max_wait);
}


/**
 * @brief Read NMEA GNS sentence
 * @param packet Pointer to NeoGPS packet structure
 * @param max_wait Maximum wait time for SPI operations
 * 
 * Reads the GNS NMEA sentence and extracts latitude and longitude.
 * @format: time,lat,NS,lon,EW,mode,numSV,HDOP,alt,sep,diffAge,diffStation*cs\r\n
 */
void read_nmea_gns(NEOM9N_t *packet, uint32_t max_wait) {
	read_nmea_gga(packet, max_wait); //pass gns data to gga handler for now
}


/**
 * @brief Read NMEA RMC sentence
 * @param packet Pointer to NeoGPS packet structure
 * @param max_wait Maximum wait time for SPI operations
 * 
 * Reads the RMC NMEA sentence and extracts latitude and longitude.
 * @format: time,status,lat,NS,lon,EW,sog,cog,date,magVar,magVarDir,posMode*cs\r\n
 */
void read_nmea_rmc(NEOM9N_t *packet, uint32_t max_wait) {
	uint8_t rx[NMEA_PAYLOAD_RX_SIZE];

	receive_nmea_payload(packet->hspi, GLOBAL_HIGH_TX, rx, NMEA_TMSTP_SIZE, max_wait);  //receive the time ( hhmmss.ss) + comma after
	receive_nmea_payload(packet->hspi, GLOBAL_HIGH_TX, rx, NMEA_STATUS_SIZE, max_wait);  //receive the status
	read_nmea_lat_and_long(packet, max_wait);   //parse the lat and long
}


/**
 * @brief Receive and process a single NMEA message over SPI
 * @param packet Pointer to NeoGPS packet structure
 * @param max_wait Maximum wait time for SPI operations
 * @param max_ignores Maximum number of messages to ignore
 * @return NEOM9N_OK if a message was processed, NEOM9N_TIMEOUT if a message was ignored
 * 
 * Receives NMEA messages over SPI and processes them based on their type.
 * Ignores messages that are not alligned.
 */
NEOM9N_status_t receive_nmea(NEOM9N_t *packet, uint32_t max_wait, uint32_t max_ignores) {
	uint8_t rx[NMEA_PAYLOAD_RX_SIZE];
	
	NEOGPS_CS_LOW(); // start transation, set cs pin to LOW
	uint32_t ignores = 0; 

	while(ignores < max_ignores) {  // ignore until we find the start of nmea message denoted by '$'
		HAL_SPI_TransmitReceive(packet->hspi, GLOBAL_HIGH_TX, rx, 1, max_wait);
		if ((char)rx[0] == '$') {
			break; //exit loop, start found
		}
		ignores ++; //start not found, continue search
	}
	if ((char)rx[0] != '$') {
		NEOGPS_CS_HIGH();
		return NEOM9N_TIMEOUT; //start not found after max ignores 
	}

	HAL_SPI_TransmitReceive(packet->hspi, GLOBAL_HIGH_TX, rx, NMEA_TALKERID_SIZE, max_wait); //pull talkerID from response 

	NEOM9N_state_t state = TAG(rx[2], rx[3], rx[4]); // pull states

	switch (state){
		case NEOM9N_GGA:
			read_nmea_gga(packet, max_wait);
			break;
		case NEOM9N_GLL:
			read_nmea_gll(packet, max_wait);
			break;
		case NEOM9N_GNS:
			read_nmea_gns(packet, max_wait);
			break;
		case NEOM9N_RMC:
			read_nmea_rmc(packet, max_wait);
			break;
		default:
			NEOGPS_CS_HIGH();
			return NEOM9N_SPI_ERR;
	}

	NEOGPS_CS_HIGH();
	return NEOM9N_OK;
}