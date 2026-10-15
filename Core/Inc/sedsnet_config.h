#pragma once

#include "sedsnet.h"

/*
 * SEDSNet v4 registers application schema entries at runtime. These stable IDs
 * derive from canonical names in config/sedsnet.json, matching SEDSnet v4
 * build-time and runtime identities regardless of file or registration order.
 * Keeping them board-owned replaces the generated v3 enum header without
 * coupling application code to the fetched dependency's source tree.
 */
#define SEDS_EP_SD_CARD ((SedsDataEndpoint)100U)
#define SEDS_EP_GROUND_STATION ((SedsDataEndpoint)191579769U)
#define SEDS_EP_FLIGHT_CONTROLLER ((SedsDataEndpoint)266702084U)
#define SEDS_EP_VALVE_BOARD ((SedsDataEndpoint)61303946U)
#define SEDS_EP_ABORT ((SedsDataEndpoint)191094473U)
#define SEDS_EP_FLIGHT_STATE ((SedsDataEndpoint)187675769U)
#define SEDS_EP_HEART_BEAT ((SedsDataEndpoint)179378936U)
#define SEDS_EP_ACTUATOR_BOARD ((SedsDataEndpoint)49912881U)
#define SEDS_EP_AV_BAY_UNDERGLOW_OWNER ((SedsDataEndpoint)26587141U)

#define SEDS_DT_GENERIC_ERROR ((SedsDataType)30636108U)
#define SEDS_DT_GPS_DATA ((SedsDataType)100U)
#define SEDS_DT_GYRO_DATA ((SedsDataType)255278604U)
#define SEDS_DT_ACCEL_DATA ((SedsDataType)210576985U)
#define SEDS_DT_BATTERY_VOLTAGE ((SedsDataType)54485088U)
#define SEDS_DT_BATTERY_CURRENT ((SedsDataType)213503347U)
#define SEDS_DT_BAROMETER_DATA ((SedsDataType)104U)
#define SEDS_DT_ABORT ((SedsDataType)135612662U)
#define SEDS_DT_FUEL_FLOW ((SedsDataType)168667933U)
#define SEDS_DT_VALVE_COMMAND ((SedsDataType)245583120U)
#define SEDS_DT_FLIGHT_COMMAND ((SedsDataType)223048036U)
#define SEDS_DT_FUEL_TANK_PRESSURE ((SedsDataType)79920485U)
#define SEDS_DT_FLIGHT_STATE ((SedsDataType)256715704U)
#define SEDS_DT_HEARTBEAT ((SedsDataType)106U)
#define SEDS_DT_WARNING ((SedsDataType)205340048U)
#define SEDS_DT_MESSAGE_DATA ((SedsDataType)105U)
#define SEDS_DT_ACTUATOR_COMMAND ((SedsDataType)218702105U)
#define SEDS_DT_UMBILICAL_STATUS ((SedsDataType)242848455U)
#define SEDS_DT_KG1000 ((SedsDataType)156609031U)
#define SEDS_DT_KG50 ((SedsDataType)11121963U)
#define SEDS_DT_GPS_SATELLITE_NUMBER ((SedsDataType)19786086U)
#define SEDS_DT_EULER_ANGLES ((SedsDataType)44028786U)
#define SEDS_DT_ORDERED_MESSAGE ((SedsDataType)51222445U)
#define SEDS_DT_ASCENT_STATE ((SedsDataType)3142600U)
#define SEDS_DT_DESCENT_STATE ((SedsDataType)117941048U)
#define SEDS_DT_ASCENT_BIASES ((SedsDataType)266798358U)
#define SEDS_DT_GYRO_LOCAL ((SedsDataType)244650351U)
#define SEDS_DT_ACCEL_LOCAL ((SedsDataType)57145916U)
#define SEDS_DT_IMU_DATA ((SedsDataType)101U)
#define SEDS_DT_IMU_LOCAL ((SedsDataType)227398105U)
#define SEDS_DT_BAROMETER_LOCAL ((SedsDataType)210488463U)
#define SEDS_DT_ASCENT_LOCAL ((SedsDataType)148442758U)
#define SEDS_DT_DESCENT_LOCAL ((SedsDataType)119099414U)
#define SEDS_DT_AV_BAY_UNDERGLOW ((SedsDataType)68266860U)
#define SEDS_DT_FLIGHT_BUZZER ((SedsDataType)169036741U)
/* Retired runtime-rate IDs remain reserved to preserve wire compatibility. */
#define SEDS_DT_RESERVED_135 ((SedsDataType)263305256U)
#define SEDS_DT_RESERVED_136 ((SedsDataType)45202657U)
#define SEDS_DT_DAQ_LOADCELL_CALIBRATION ((SedsDataType)189079860U)
