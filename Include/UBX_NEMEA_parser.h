#ifndef UBX_NEMEA_PARSER
#define UBX_NEMEA_PARSER

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    UBX_NAV_NO_FIX = 0,
    UBX_NAV_DEAD_RECKONING_ONLY = 1,
    UBX_NAV_2D_FIX = 2,
    UBX_NAV_3D_FIX = 3,
    UBX_NAV_GNSS_DEAD_RECKONING_COMBINED = 4,
    UBX_NAV_TIME_ONLY_FIX = 5,
    UBX_NAV_UNKNOWN = 255
} ubx_nav_type_t;

typedef struct
{
    // Latitude in NMEA format: GGMM.MM
    float latitude_nmea;

    // Latitude in decimal degrees
    float latitude_deg;

    // Longitude in NMEA format: GGGMM.MM
    float longitude_nmea;

    // Longitude in decimal degrees
    float longitude_deg;

    // Altitude in meters
    float altitude;

    // Raw UTC time in HHMMSS.SS format
    float time;

    ubx_nav_type_t nav_type;
    uint8_t satellite_count;
    uint8_t fix_3d;

    // Accuracy in meters
    float HorizontalAccuracy;
    float verticalAccuracy;

    // Speed in meters per second
    float horizontalSpeed;
    float verticalSpeed;

    // Course Over Ground (COG) in degrees.
    // Represents the direction of movement relative to true north.
    float COG;

    // Horizontal Dilution of Precision (HDOP).
    // Indicates horizontal position accuracy (lower value = better accuracy).
    float hDOP;

    // Vertical Dilution of Precision (VDOP).
    // Indicates vertical position accuracy (lower value = better accuracy).
    float vDOP;

    // Time Dilution of Precision (TDOP).
    // Indicates time/clock accuracy of the GPS solution (lower value = better accuracy).
    float tDOP;
} ubx_gps_data_t;

// Parses an NMEA buffer using a finite state machine.
void ubx_parseNMEA(uint8_t* gpsBuffer, uint16_t size);

// Parses one byte of an NMEA stream.
void ubx_parseNMEA_char(unsigned char s);

ubx_gps_data_t ubx_getAll();
uint8_t ubx_hasNewData();

// Returns latitude in decimal degrees.
// South latitude is represented as a negative value.
float ubx_getLat_deg();

// Returns latitude in NMEA format: GGMM.MM.
float ubx_getLat_nmea();

// Returns longitude in decimal degrees.
// West longitude is represented as a negative value.
float ubx_getLon_deg();

// Returns longitude in NMEA format: GGGMM.MM.
float ubx_getLon_nmea();

// Returns raw UTC time in HHMMSS.SS format.
float ubx_getTime();

// meters.
float ubx_getAltitude();

// Returns the number of tracked satellites.
float ubx_getSatelliteCount();

// Returns the current navigation type.
// See ubx_nav_type_t for possible values.
ubx_nav_type_t ubx_getNavigationType();

uint8_t ubx_has3DFix();

// meters per second
float ubx_getHorizontalSpeed();

// meters per second
float ubx_getVerticalSpeed();

// accuracy in meters
float ubx_getHorizontalAccuracy();

// accuracy in meters
float ubx_getVerticalAccuracy();

// Course Over Ground (COG) in degrees.
// Represents the direction of movement relative to true north.
float ubx_getCOG();

// Horizontal Dilution of Precision (HDOP).
// Indicates horizontal position accuracy (lower value = better accuracy).
float ubx_getHDOP();

// Vertical Dilution of Precision (VDOP).
// Indicates vertical position accuracy (lower value = better accuracy).
float ubx_getVDOP();

// Time Dilution of Precision (TDOP).
// Indicates time/clock accuracy of the GPS solution (lower value = better accuracy).
float ubx_getTDOP();

#ifdef __cplusplus
}
#endif

#endif // UBX_NEMEA_PARSER
