/******************************************************************************
 * @file    neom9n.h
 * @author  John Welgoss
 * @version V1.0
 * @date    11-11-2025
 * @brief   NEO-M9N GPS Driver Header File
 ******************************************************************************
 * @note
 * 
 * This file contains the function prototypes and definitions for interfacing
 * with the NEO-M9N GPS module.
 * 
 * Includes all necessary libraries.
 * Includes all necessary definitions for parsing GPS data.
 * Includes helper functions to parse latitude and longitude from the GPS data.
 */


#ifndef __CORE_INC_DRIVERS_NEOM9N_H 
#define __CORE_INC_DRIVERS_NEOM9N_H  // Define to prevent recursive inclusion 

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"

// Helper function prototypes
float parseLat(uint8_t *rx);  // Function to parse latitude from NEO-M9N GPS data
float parseLon(uint8_t *rx);  // Function to parse longitude from NEO-M9N GPS data








#endif /* __CORE_INC_DRIVERS_NEOM9N_H */