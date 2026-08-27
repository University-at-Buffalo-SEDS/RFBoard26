#pragma once

#include "sedsnet.h"

/*
 * SEDSNet v4 registers application schema entries at runtime. These stable IDs
 * match config/sedsnet.json (entries are assigned sequentially from 100).
 * Keeping them board-owned replaces the generated v3 enum header without
 * coupling application code to the fetched dependency's source tree.
 */
#define SEDS_EP_SD_CARD ((SedsDataEndpoint)100U)
#define SEDS_EP_GROUND_STATION ((SedsDataEndpoint)101U)
#define SEDS_EP_FLIGHT_CONTROLLER ((SedsDataEndpoint)102U)
#define SEDS_EP_VALVE_BOARD ((SedsDataEndpoint)103U)
#define SEDS_EP_ABORT ((SedsDataEndpoint)104U)
#define SEDS_EP_FLIGHT_STATE ((SedsDataEndpoint)105U)
#define SEDS_EP_HEART_BEAT ((SedsDataEndpoint)106U)
#define SEDS_EP_ACTUATOR_BOARD ((SedsDataEndpoint)107U)

#define SEDS_DT_GENERIC_ERROR ((SedsDataType)100U)
#define SEDS_DT_GPS_DATA ((SedsDataType)101U)
#define SEDS_DT_GYRO_DATA ((SedsDataType)102U)
#define SEDS_DT_ACCEL_DATA ((SedsDataType)103U)
#define SEDS_DT_BATTERY_VOLTAGE ((SedsDataType)104U)
#define SEDS_DT_BATTERY_CURRENT ((SedsDataType)105U)
#define SEDS_DT_BAROMETER_DATA ((SedsDataType)106U)
#define SEDS_DT_ABORT ((SedsDataType)107U)
#define SEDS_DT_FUEL_FLOW ((SedsDataType)108U)
#define SEDS_DT_VALVE_COMMAND ((SedsDataType)109U)
#define SEDS_DT_FLIGHT_COMMAND ((SedsDataType)110U)
#define SEDS_DT_FUEL_TANK_PRESSURE ((SedsDataType)111U)
#define SEDS_DT_FLIGHT_STATE ((SedsDataType)112U)
#define SEDS_DT_HEARTBEAT ((SedsDataType)113U)
#define SEDS_DT_WARNING ((SedsDataType)114U)
#define SEDS_DT_MESSAGE_DATA ((SedsDataType)115U)
#define SEDS_DT_ACTUATOR_COMMAND ((SedsDataType)116U)
#define SEDS_DT_UMBILICAL_STATUS ((SedsDataType)117U)
#define SEDS_DT_KG1000 ((SedsDataType)118U)
#define SEDS_DT_KG50 ((SedsDataType)119U)
#define SEDS_DT_GPS_SATELLITE_NUMBER ((SedsDataType)120U)
#define SEDS_DT_EULER_ANGLES ((SedsDataType)121U)
#define SEDS_DT_ORDERED_MESSAGE ((SedsDataType)122U)
#define SEDS_DT_ASCENT_STATE ((SedsDataType)123U)
#define SEDS_DT_DESCENT_STATE ((SedsDataType)124U)
#define SEDS_DT_ASCENT_BIASES ((SedsDataType)125U)
#define SEDS_DT_GYRO_LOCAL ((SedsDataType)126U)
#define SEDS_DT_ACCEL_LOCAL ((SedsDataType)127U)
#define SEDS_DT_IMU_DATA ((SedsDataType)128U)
#define SEDS_DT_IMU_LOCAL ((SedsDataType)129U)
#define SEDS_DT_BAROMETER_LOCAL ((SedsDataType)130U)
#define SEDS_DT_ASCENT_LOCAL ((SedsDataType)131U)
#define SEDS_DT_DESCENT_LOCAL ((SedsDataType)132U)
