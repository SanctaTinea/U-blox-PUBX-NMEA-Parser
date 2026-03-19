#include <stdint.h>
#include <stdio.h>
#include "UBX_NEMEA_parser.h"

/*
 * Example: using the u-blox PUBX NMEA parser with UART DMA (STM32-style).
 *
 * The parser processes incoming GPS data in a streaming manner.
 * Data is received via DMA into a buffer and parsed in chunks
 * using DMA half-transfer and transfer-complete callbacks.
 */

/* --- DMA buffer configuration --- */

#define GPS_BUFFER_SIZE 128
uint8_t gpsBuffer[GPS_BUFFER_SIZE];

/*
 * Called when the first half of the DMA buffer is filled.
 * Parse the first half of the buffer.
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	ubx_parseNMEA(gpsBuffer, GPS_BUFFER_SIZE / 2);
}

/*
 * Called when the full DMA buffer is filled.
 * Parse the second half of the buffer.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	ubx_parseNMEA(gpsBuffer + GPS_BUFFER_SIZE / 2, GPS_BUFFER_SIZE / 2);
}

/* --- Example processing function --- */
void process_gps_data(const ubx_gps_data_t *data)
{
    printf("Latitude:  %.6f deg\n", data->latitude_deg);
    printf("Longitude: %.6f deg\n", data->longitude_deg);
    printf("Altitude:  %.2f m\n", data->altitude);

    printf("Satellites: %u\n", data->satellite_count);

    printf("Speed: H=%.2f m/s, V=%.2f m/s\n",
           data->horizontalSpeed,
           data->verticalSpeed);

    printf("Accuracy: H=%.2f m, V=%.2f m\n",
           data->HorizontalAccuracy,
           data->verticalAccuracy);

    printf("COG: %.2f deg\n", data->COG);

    printf("DOP: HDOP=%.2f, VDOP=%.2f, TDOP=%.2f\n",
           data->hDOP,
           data->vDOP,
           data->tDOP);

    printf("3D Fix: %s\n", data->fix_3d ? "YES" : "NO");
    printf("----------------------------------------\n");
}

/* --- Main example --- */
int main(void)
{
    /*
     * Start UART reception using DMA.
     * The buffer will be filled continuously in circular mode.
     */
    HAL_UART_Receive_DMA(&huartx, gpsBuffer, GPS_BUFFER_SIZE);

    while (1)
    {
        /*
         * Check if a new valid GPS message has been parsed.
         * ubx_hasNewData() is set after successful checksum validation.
         */
        if (ubx_hasNewData())
        {
            ubx_gps_data_t data = ubx_getAll();

            // Process parsed GPS data
            process_gps_data(&data);
        }
    }
}