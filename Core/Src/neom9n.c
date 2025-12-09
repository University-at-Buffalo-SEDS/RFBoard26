/** 
 * @file    neom9n.c
 * @author  John Welgoss
 * @brief   NEO-M9N GPS Driver File
 */

 #include "neom9n.h"
 #include "stm32g4xx_hal_def.h"
 #include "stm32g4xx_hal_spi.h"
 //#include "telemetry.h"
 #include <stdint.h>
 #include <stdbool.h>
 #include <math.h>
 #include <string.h>
 #include <stdatomic.h>

// Globals 
#define TAG(a,b,c) ((a << 16) | (b << 8) | (c))  //Packs size 3 str for swtich-case
uint8_t GLOBAL_HIGH_TX[SPI_RX_BUFFER_SIZE] = {[0 ... SPI_RX_BUFFER_SIZE-1] = 0xFF}; // Global TX buffer filled with 0xFF for SPI reads


/**
 * @brief Parse Latitude from NMEA payload
 * @param payload Pointer to NMEA payload containing latitude
 * @return Latitude in decimal degrees
 * 
 * Format: ddmm.mmmmm
 */
float parseLat(uint8_t *payload){
	int deg = char_to_int(payload[0]) * 10 + char_to_int(payload[1]); // degrees part
	float min = 0;  // minutes part
	float pos = 10;  // position value for minutes calculation

	for (int i = 0; i < 8; i++) { 
		if(i != 2) {  //skip decimal point
			min += pos * char_to_int(payload[2+i]); 
			pos = pos/10; //new position value
		}
	}

	return (float)deg + min/60;
}

/**
 * @brief Parse Longitude from NMEA payload
 * @param payload Pointer to NMEA payload containing longitude
 * @return Longitude in decimal degrees
 * 
 * Format: dddmm.mmmmm
 */
float parseLong(uint8_t *payload) {
	int deg = char_to_int(payload[0]) * 100 + char_to_int(payload[1]) * 10 + char_to_int(payload[2]); // degrees part
	float min = 0;  // minutes part
	float pos = 10;  // position value for minutes calculation

	for (int i = 0; i < 8; i++) { 
		if(i != 2) {   //skip decimal point
			min += pos * char_to_int(payload[3+i]);  
			pos = pos/10;   //new position value
		}
	}

	return (float)deg + min/60;
}

/**
 * @brief Receive a section of NMEA payload until a comma or size bytes
 * @param hspi Pointer to SPI handle
 * @param tx Pointer to TX buffer
 * @param rx Pointer to RX buffer
 * @param size Maximum number of bytes to read
 * @param max_wait Maximum wait time for SPI operations
 * @return true if data was read, false if a comma was encountered first
 * 
 * Reads bytes into rx until a comma is encountered or size bytes have been read.
 * The first byte is checked to see if it's a comma; if so, no further bytes are read.
 */
bool receive_nmea_payload(SPI_HandleTypeDef *hspi, uint8_t *tx, uint8_t *rx, uint16_t size, uint32_t max_wait) { 
	//stop at ',' or size bytes
	HAL_SPI_TransmitReceive(hspi, tx, rx, 1, max_wait);  //read first byte 
	
	if (rx[0] == ',') {
		return false;   //if first byte is comma, stop reading
	}

	HAL_SPI_TransmitReceive(hspi, tx, rx+1, size-1, max_wait); 	//otherwise continue to read the rest into rx

	return true;
}

/**
 * @brief Read latitude and longitude from NMEA payload
 * @param config Pointer to NeoGPS configuration structure
 * @param max_wait Maximum wait time for SPI operations
 * 
 * Reads latitude and longitude from the NMEA payload and updates the config struct.
 */
void read_nmea_lat_and_long(NEOM9N_t *config, uint32_t max_wait) {
	uint8_t rx[NMEA_PAYLOAD_RX_SIZE]; //buffer for receiving nmea payload sections

	//receive the latitude(ddmm.mmmmm)+ comma after
	float recv_lat = config->latitude_deg;
	if (receive_nmea_payload(config->hspi, GLOBAL_HIGH_TX, rx, NMEA_LATT_SIZE, max_wait) == true) {
		recv_lat = parseLat(rx);
	}

	//receive the NS indicator and comma after
	uint8_t ns_indicator;
	if (receive_nmea_payload(config->hspi, GLOBAL_HIGH_TX, &ns_indicator, NMEA_NS_SIZE, max_wait) == true) {
		config->latitude_deg = (ns_indicator == 'N' ? recv_lat : -recv_lat);
	}

	//receive the longitude(dddmm.mmmmm)+ comma after (one more d than latitude)
	float recv_long = config->longitude_deg;
	if (receive_nmea_payload(config->hspi, GLOBAL_HIGH_TX, rx, NMEA_LONG_SIZE, max_wait) == true) {
		recv_long = parseLong(rx);
	}

	//receive the EW indicator and comma after
	uint8_t ew_indicator;
	if (receive_nmea_payload(config->hspi, GLOBAL_HIGH_TX, &ew_indicator, NMEA_EW_SIZE, max_wait) == true) {
		config->longitude_deg = (ew_indicator == 'E' ? recv_long : -recv_long);
	}
}

/**
 * @brief Read NMEA GGA sentence, and NMEA GNS sentence
 * @param config Pointer to NeoGPS configuration structure
 * @param max_wait Maximum wait time for SPI operations
 * 
 * Reads the GGA NMEA sentence and extracts latitude and longitude.
 * Format: time,lat,NS,lon,EW,fixQuality,numSV,HDOP,alt,sep,diffAge,diffStation,navStatus*cs\r\n
 */
void read_nmea_gga(NEOM9N_t *config, uint32_t max_wait) {
	uint8_t rx[NMEA_PAYLOAD_RX_SIZE];

	//receive the time ( hhmmss.ss) + comma after
	receive_nmea_payload(config->hspi, GLOBAL_HIGH_TX, rx, NMEA_TMSTP_SIZE, max_wait);

	read_nmea_lat_and_long(config, max_wait);
}

/**
 * @brief Read NMEA GLL sentence
 * @param config Pointer to NeoGPS configuration structure
 * @param max_wait Maximum wait time for SPI operations
 * 
 * Reads the GLL NMEA sentence and extracts latitude and longitude.
 * Format: lat,NS,lon,EW,time,status,posMode*cs\r\n
 */
void read_nmea_gll(NEOM9N_t *config, uint32_t max_wait) {
	read_nmea_lat_and_long(config, max_wait);
}

/**
 * @brief Read NMEA GNS sentence
 * @param config Pointer to NeoGPS configuration structure
 * @param max_wait Maximum wait time for SPI operations
 * 
 * Reads the GNS NMEA sentence and extracts latitude and longitude.
 * Format: time,lat,NS,lon,EW,mode,numSV,HDOP,alt,sep,diffAge,diffStation*cs\r\n
 */
void read_nmea_gns(NEOM9N_t *config, uint32_t max_wait) {
	read_nmea_gga(config, max_wait);
}

/**
 * @brief Read NMEA RMC sentence
 * @param config Pointer to NeoGPS configuration structure
 * @param max_wait Maximum wait time for SPI operations
 * 
 * Reads the RMC NMEA sentence and extracts latitude and longitude.
 * Format: time,status,lat,NS,lon,EW,sog,cog,date,magVar,magVarDir,posMode*cs\r\n
 */
void read_nmea_rmc(NEOM9N_t *config, uint32_t max_wait) {
	uint8_t rx[NMEA_PAYLOAD_RX_SIZE];

	receive_nmea_payload(config->hspi, GLOBAL_HIGH_TX, rx, NMEA_TMSTP_SIZE, max_wait);  //receive the time ( hhmmss.ss) + comma after
	receive_nmea_payload(config->hspi, GLOBAL_HIGH_TX, rx, NMEA_STATUS_SIZE, max_wait);  //receive the status
	read_nmea_lat_and_long(config, max_wait);   //parse the lat and long
}

/**
 * @brief Receive and process a single NMEA message over SPI
 * @param config Pointer to NeoGPS configuration structure
 * @param max_wait Maximum wait time for SPI operations
 * @param max_ignores Maximum number of messages to ignore
 * @return true if a message was processed, false if a message was ignored
 * 
 * Receives NMEA messages over SPI and processes them based on their type.
 * Ignores messages that are not alligned.
 */
bool receive_nmea(NEOM9N_t *config, uint32_t max_wait, uint32_t max_ignores) {
	uint8_t *pRx[NMEA_PAYLOAD_RX_SIZE];
	
	NEOGPS_CS_LOW(); // start transation, set cs pin to LOW
	uint32_t ignores = 0; 

	while(ignores < max_ignores) {  // ignore until we find the start of nmea message denoted by '$'
		HAL_SPI_TransmitReceive(config->hspi, GLOBAL_HIGH_TX, pRx, 1, max_wait);
		if ((char)*pRx[0] == '$') {
			break; //exit loop, start found
		}
		ignores ++; //start not found, continue search
	}
	if ((char)*pRx[0] != '$') {
		NEOGPS_CS_HIGH();
		return false; //start not found after max ignores 
	}

	HAL_SPI_TransmitReceive(config->hspi, GLOBAL_HIGH_TX, pRx, NMEA_TALKERID_SIZE, max_wait); //pull talkerID from response 

	char *pSent = (char*)pRx + 2;
	uint32_t tag = TAG(pSent[0], pSent[1], pSent[2]); // contruct tag

	switch(tag) {
		case TAG('G','G','A'):
			read_nmea_gga(config, max_wait);
			//print statement
			break;
		case TAG('G','L','L'):
			read_nmea_gll(config, max_wait);
			//print statement 
			break;
		case TAG('G','N','S'):
			read_nmea_gns(config, max_wait);
			//print statement
			break;
		case TAG('R','M','C'):
			read_nmea_rmc(config, max_wait);
			//print statement 
			break;
		default:
			NEOGPS_CS_HIGH();
			//print statement
			return false;
	}

	NEOGPS_CS_HIGH();
	return true;
}
