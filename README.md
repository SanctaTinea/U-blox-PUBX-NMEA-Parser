# U-blox PUBX NMEA Parser

Lightweight streaming NMEA parser for u-blox GPS modules.

Designed for embedded systems (STM32, bare-metal, RTOS) with no dynamic memory allocation.

---

## ✨ Features

- Finite State Machine (FSM) based parser
- Supports streaming input (byte-by-byte parsing)
- No dynamic memory allocation
- Checksum validation
- Extracts GPS data (position, speed, accuracy, DOP, etc.)
- Optimized for low-resource embedded systems

---

## Available Data

- Latitude (NMEA + decimal degrees)
- Longitude (NMEA + decimal degrees)
- Altitude
- Time (UTC)
- Navigation type (2D/3D fix and more)
- Satellite count
- Speed (horizontal / vertical)
- Accuracy (horizontal / vertical)
- DOP values (HDOP, VDOP, TDOP)
- Course Over Ground (COG)

---

## ⚠️ Limitations

- Supports only selected **u-blox PUBX messages**
- Not a full generic NMEA parser
- Single parser instance (non-reentrant)

---

## 🚀 Usage

### Parse a byte stream and read data as `ubx_gps_data_t`

```c
#include "UBX_NEMEA_parser.h"

int main(void)
{
    uint8_t byte;

    while (1)
    {
        byte = uart_read_byte();
        ubx_parseNMEA(&byte, 1); // (uint8_t* gpsBuffer, uint16_t size)

        if (ubx_hasNewData())
        {
            ubx_gps_data_t data = ubx_getAll();

            // Use parsed data
            float lat = data.latitude_deg;
            float lon = data.longitude_deg;
            // ...
        }
    }
}
```

### Or use getters

```c
#include "UBX_NEMEA_parser.h"

int main(void)
{
    uint8_t byte;

    while (1)
    {
        byte = uart_read_byte();
        ubx_parseNMEA_char(byte); // you can use _char version of function

        // Internal value will be updated automatically
        // getters return the latest parsed values
        float lat = ubx_getLat_deg();
        float lon = ubx_getLon_deg();
        // ...
    }
}
```