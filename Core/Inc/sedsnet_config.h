#pragma once

#include <stdint.h>
#ifdef TELEMETRY_ENABLED
#include "sedsnet.h"
#endif

/*
 * SEDSNet v4 registers application schema entries at runtime. These stable IDs
 * derive from canonical names in config/sedsnet.json, matching SEDSnet v4
 * build-time and runtime identities regardless of file or registration order.
 * Keeping them board-owned replaces the generated v3 enum header without
 * coupling application code to the fetched dependency's source tree.
 */
#define SEDS_EP_SD_CARD ((uint32_t)100U)
#define SEDS_EP_GROUND_STATION ((uint32_t)191579769U)
#define SEDS_EP_FLIGHT_CONTROLLER ((uint32_t)266702084U)
#define SEDS_EP_VALVE_BOARD ((uint32_t)61303946U)
#define SEDS_EP_ABORT ((uint32_t)191094473U)
#define SEDS_EP_FLIGHT_STATE ((uint32_t)187675769U)
#define SEDS_EP_HEART_BEAT ((uint32_t)179378936U)
#define SEDS_EP_ACTUATOR_BOARD ((uint32_t)49912881U)
#define SEDS_EP_AV_BAY_UNDERGLOW_OWNER ((uint32_t)26587141U)

#define SEDS_DT_GENERIC_ERROR ((uint32_t)30636108U)
#define SEDS_DT_GPS_DATA ((uint32_t)100U)
#define SEDS_DT_GYRO_DATA ((uint32_t)255278604U)
#define SEDS_DT_ACCEL_DATA ((uint32_t)210576985U)
#define SEDS_DT_BATTERY_VOLTAGE ((uint32_t)54485088U)
#define SEDS_DT_BATTERY_CURRENT ((uint32_t)213503347U)
#define SEDS_DT_BAROMETER_DATA ((uint32_t)104U)
#define SEDS_DT_ABORT ((uint32_t)135612662U)
#define SEDS_DT_FUEL_FLOW ((uint32_t)168667933U)
#define SEDS_DT_VALVE_COMMAND ((uint32_t)245583120U)
#define SEDS_DT_FLIGHT_COMMAND ((uint32_t)223048036U)
#define SEDS_DT_FUEL_TANK_PRESSURE ((uint32_t)79920485U)
#define SEDS_DT_FLIGHT_STATE ((uint32_t)256715704U)
#define SEDS_DT_HEARTBEAT ((uint32_t)106U)
#define SEDS_DT_WARNING ((uint32_t)205340048U)
#define SEDS_DT_MESSAGE_DATA ((uint32_t)105U)
#define SEDS_DT_ACTUATOR_COMMAND ((uint32_t)218702105U)
#define SEDS_DT_UMBILICAL_STATUS ((uint32_t)242848455U)
#define SEDS_DT_KG1000 ((uint32_t)156609031U)
#define SEDS_DT_KG50 ((uint32_t)11121963U)
#define SEDS_DT_GPS_SATELLITE_NUMBER ((uint32_t)19786086U)
#define SEDS_DT_EULER_ANGLES ((uint32_t)44028786U)
#define SEDS_DT_ORDERED_MESSAGE ((uint32_t)51222445U)
#define SEDS_DT_ASCENT_STATE ((uint32_t)3142600U)
#define SEDS_DT_DESCENT_STATE ((uint32_t)117941048U)
#define SEDS_DT_ASCENT_BIASES ((uint32_t)266798358U)
#define SEDS_DT_GYRO_LOCAL ((uint32_t)244650351U)
#define SEDS_DT_ACCEL_LOCAL ((uint32_t)57145916U)
#define SEDS_DT_IMU_DATA ((uint32_t)101U)
#define SEDS_DT_IMU_LOCAL ((uint32_t)227398105U)
#define SEDS_DT_BAROMETER_LOCAL ((uint32_t)210488463U)
#define SEDS_DT_ASCENT_LOCAL ((uint32_t)148442758U)
#define SEDS_DT_DESCENT_LOCAL ((uint32_t)119099414U)
#define SEDS_DT_AV_BAY_UNDERGLOW ((uint32_t)68266860U)
#define SEDS_DT_FLIGHT_BUZZER ((uint32_t)169036741U)
/* Retired runtime-rate IDs remain reserved to preserve wire compatibility. */
#define SEDS_DT_RESERVED_135 ((uint32_t)263305256U)
#define SEDS_DT_RESERVED_136 ((uint32_t)45202657U)
#define SEDS_DT_DAQ_LOADCELL_CALIBRATION ((uint32_t)189079860U)

#define SEDS_DT_BOARD_HEALTH ((uint32_t)215170867U)
