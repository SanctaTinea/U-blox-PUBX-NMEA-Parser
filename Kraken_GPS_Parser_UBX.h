#ifndef KRAKEN_GNS_PARSER
#define KRAKEN_GNS_PARSER

#include <stdint.h>

namespace Kraken {
// Полученные значения
// GGMM.MM
extern double gpsLat;
// GGGMM.MM
extern double gpsLon;
// MM.MM - метры
extern float gpsH;
// HHMMSS.SS
extern float gpsTime;
// Направление широты и долготы
extern uint8_t gpsVector;
// Были ли хоть раз получены ккординаты, true = да
extern bool gpsReady;
// Количество 'пойманных' спутников
extern uint8_t gpsStlCount;

/* Сложение целой и дробной частей вместе */
double addToDuoble(uint32_t m, uint32_t l);

/* NMEA парсер основанный на конечном автомате.
   gpsBuffer - указатель на буфер, size - его размер  */
void parseNMEA(uint8_t* gpsBuffer, uint16_t size);

/* NMEA парсер основанный на конечном автомате.
   Можно посимвольно отправлять данные в парсер  */
void parseNMEA(unsigned char s);

/* Возврат широту в формате GG.GG
   Если южная, то значение будет отрицательным */
float getLat();

/* Возврат долготу в формате GGG.GG
   Если западная, то значение будет отрицательным */
float getLon();

/* Время в формате ЧЧММСС.СС */
float getTime();

/* Высота в метрах */
float getAltitude();

/* Количество 'пойманных' спутникоа */
float getStlCount();

float getNavigationType();
bool if3DFixAvailable();

float getSpeedHorizontal();
float getSpeedVertical();

float getAccHorizontal();
float getAccVertical();

float getCOG();
float getHDOP();
float getVDOP();
float getTDOP();
}

#endif // KRAKEN_GNS_PARSER
