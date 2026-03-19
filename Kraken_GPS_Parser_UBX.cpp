#include "Kraken_GPS_Parser_UBX.h"

namespace ubx_nem {
// Временное хранение данных и состояний автомата для парсера
uint8_t gpsTitleBuf[5];
int8_t parserState = -1;
int8_t gpsElemType = 0;
uint16_t gpsLat_m = 0;
uint32_t gpsLat_l = 0;
uint16_t gpsLon_m = 0;
uint32_t gpsLon_l = 0;
uint32_t gpsH_m = 0;
uint32_t gpsH_l = 0;
uint32_t gpsTime_m = 0;
uint32_t gpsTime_l = 0;

uint8_t gpsMsgID = 0;

uint8_t gpsNavStat_buf[2];
uint32_t hAcc_m = 0;
uint16_t hAcc_l = 0;
uint32_t vAcc_m = 0;
uint16_t vAcc_l = 0;
uint16_t speed_m = 0;
uint16_t speed_l = 0;
int8_t speed_p = 1;
uint16_t COG_m = 0;
uint16_t COG_l = 0;
uint16_t speedV_m = 0;
uint16_t speedV_l = 0;
int8_t speedV_p = 1;
uint16_t HDOT_m = 0;
uint16_t HDOT_l = 0;
uint16_t VDOT_m = 0;
uint16_t VDOT_l = 0;
uint16_t TDOT_m = 0;
uint16_t TDOT_l = 0;

uint8_t gpsVector_buf = 0;
uint8_t gpsStlCount_buf = 0;
uint8_t gpsXurSum = 0;
uint8_t gpsCtlSumBuf = 0;

// Полученные значения
float gpsH = 0;
double gpsLat = 0;
double gpsLon = 0;
float gpsTime = 0;
uint8_t gpsVector = 0;
uint8_t gpsStlCount = 0;
bool gpsReady = false;

uint8_t gpsNavStat = 0;
float hAcc = 0;
float vAcc = 0;
float speed = 0;
float COG = 0;
float speedV = 0;
float HDOT = 0;
float VDOT = 0;
float TDOT = 0;

void parseNMEA(unsigned char s) { parseNMEA(&s, 1); }

void parseNMEA(uint8_t *gpsBuffer, uint16_t size) {
	for (uint16_t i = 0; i < size; ++i) {
		if (gpsBuffer[i] == '$') { // Начало нового пакета
			parserState = 0;
			gpsElemType = 0;
			gpsLat_m = 0; gpsLat_l = 0;
			gpsLon_m = 0; gpsLon_l = 0;
			gpsH_m = 0; gpsH_l = 0;
			gpsTime_m = 0; gpsTime_l = 0;
			gpsVector_buf = 0;
			gpsCtlSumBuf = 0;
			gpsXurSum = 0;
			gpsStlCount_buf = 0;

            gpsMsgID = 0;
            gpsNavStat_buf[0] = 0;
            gpsNavStat_buf[1] = 0;
			hAcc_m = 0;
            hAcc_l = 0;
            vAcc_m = 0;
            vAcc_l = 0;
            speed_m = 0;
            speed_l = 0;
            COG_m = 0;
            COG_l = 0;
            speedV_m = 0;
            speedV_l = 0;
            HDOT_m = 0;
            HDOT_l = 0;
            VDOT_m = 0;
            VDOT_l = 0;
            TDOT_m = 0;
            TDOT_l = 0;
			continue;
		}

		if (parserState == -1) { continue; }

		// Конец пакета
		if (gpsBuffer[i] == '\r') {
			parserState = -1;
			if (gpsCtlSumBuf == gpsXurSum) { // Если контрольная сумма совпала
				// Обновление данных
				gpsLat = addToDuoble(gpsLat_m, gpsLat_l);
				gpsLon = addToDuoble(gpsLon_m, gpsLon_l);
				gpsH = addToDuoble(gpsH_m, gpsH_l);
				gpsTime = addToDuoble(gpsTime_m, gpsTime_l);
				gpsVector = gpsVector_buf;
				gpsStlCount = gpsStlCount_buf;
				gpsNavStat = 0;
				if (gpsNavStat_buf[0] == 'N' && gpsNavStat_buf[1] == 'F') { gpsNavStat = 0; }
				if (gpsNavStat_buf[0] == 'D' && gpsNavStat_buf[1] == 'R') { gpsNavStat = 1; }
				if (gpsNavStat_buf[0] == 'G' && gpsNavStat_buf[1] == '2') { gpsNavStat = 2; }
				if (gpsNavStat_buf[0] == 'G' && gpsNavStat_buf[1] == '3') { gpsNavStat = 3; }
				if (gpsNavStat_buf[0] == 'D' && gpsNavStat_buf[1] == '2') { gpsNavStat = 4; }
				if (gpsNavStat_buf[0] == 'D' && gpsNavStat_buf[1] == '3') { gpsNavStat = 5; }
				if (gpsNavStat_buf[0] == 'R' && gpsNavStat_buf[1] == 'K') { gpsNavStat = 6; }
				if (gpsNavStat_buf[0] == 'T' && gpsNavStat_buf[1] == 'T') { gpsNavStat = 7; }
				hAcc = addToDuoble(hAcc_m, hAcc_l);
				vAcc = addToDuoble(vAcc_m, vAcc_l);
				speed = addToDuoble(speed_m, speed_l)*speed_p;
				speedV = addToDuoble(speedV_m, speedV_l)*speedV_p;
				COG = addToDuoble(COG_m, COG_l);
				HDOT = addToDuoble(HDOT_m, HDOT_l);
				VDOT = addToDuoble(VDOT_m, VDOT_l);
				TDOT = addToDuoble(TDOT_m, TDOT_l);

				// Если gpsReady == true, то GPS 'поймал' стуники
				if (gpsLat != 0.0 && gpsLon != 0.0) { gpsReady = true; }
			}
			continue;
		}

		if (parserState == -2) { // Если сейчас передаётся контрольная сумма
			if (gpsBuffer[i] >= 'A') {
				// Вычитаем 7 из-за семи символов между цифрами и буквами в ASCII
				gpsCtlSumBuf = gpsCtlSumBuf*16 + gpsBuffer[i]-'0'-7;
			}
			else { gpsCtlSumBuf = gpsCtlSumBuf*16 + gpsBuffer[i]-'0'; }
			continue;
		}

		// Начало контрольной суммы с GPS
		if (gpsBuffer[i] == '*') { parserState = -2; continue; }

		// Вычисление контрольной суммы
		gpsXurSum ^= gpsBuffer[i];

		if (parserState == 0) { // Ожидание нужных заголовков
			if (gpsBuffer[i] == ',') {
				if (gpsTitleBuf[1] == 'P' && gpsTitleBuf[2] == 'U' && gpsTitleBuf[3] == 'B' && gpsTitleBuf[4] == 'X') {
					parserState = 1;
					continue;
				}
				parserState = -1;
				continue;
			}
			// Обновление послдених пяти символов
			gpsTitleBuf[0] = gpsTitleBuf[1];
			gpsTitleBuf[1] = gpsTitleBuf[2];
			gpsTitleBuf[2] = gpsTitleBuf[3];
			gpsTitleBuf[3] = gpsTitleBuf[4];
			gpsTitleBuf[4] = gpsBuffer[i];
			continue;
		}

		if (parserState == 1) { // Парсинг нужного сообщение
			if (gpsBuffer[i] == ',' || gpsBuffer[i] == '.') {

                if (gpsElemType == 0 && gpsMsgID != 0) { parserState = -1; continue; }

				// Если пришла точка или запятая, но переходим к следующему значению
				if (gpsBuffer[i] == ',') {
					if (gpsElemType == 1 || gpsElemType == 3 || gpsElemType == 6 ||
					gpsElemType == 9 || gpsElemType == 12 || gpsElemType == 14 ||
					gpsElemType == 16 || gpsElemType == 18 || gpsElemType == 20 ||
					gpsElemType == 22 || gpsElemType == 24 || gpsElemType == 26 ||
					gpsElemType == 28) { gpsElemType++; }
				}
				gpsElemType++;
				continue;
			}
			// (gpsElemType) - номер текущего значения
			if (gpsElemType == 0) { gpsMsgID = gpsMsgID*10 + gpsBuffer[i]-'0'; continue; }
			if (gpsElemType == 1) { gpsTime_m = gpsTime_m*10 + gpsBuffer[i]-'0'; continue; }
			if (gpsElemType == 2) { gpsTime_l = gpsTime_l*10 + gpsBuffer[i]-'0'; continue; }
			if (gpsElemType == 3) { gpsLat_m = gpsLat_m*10 + gpsBuffer[i]-'0'; continue; }
			if (gpsElemType == 4) { gpsLat_l = gpsLat_l*10 + gpsBuffer[i]-'0'; continue; }
			if (gpsElemType == 5) {
				gpsVector_buf = (gpsVector_buf&0b11) | 0b10;
				if (gpsBuffer[i] == 'S') { gpsVector_buf = gpsVector_buf&0b01; }
				continue;
			}
			if (gpsElemType == 6) { gpsLon_m = gpsLon_m*10 + gpsBuffer[i]-'0'; continue; }
			if (gpsElemType == 7) { gpsLon_l = gpsLon_l*10 + gpsBuffer[i]-'0'; continue; }
			if (gpsElemType == 8) {
				gpsVector_buf = (gpsVector_buf&0b11) | 0b01;
				if (gpsBuffer[i] == 'W') { gpsVector_buf = gpsVector_buf&0b10; }
				continue;
			}
			if (gpsElemType == 9) { gpsH_m = gpsH_m*10 + gpsBuffer[i]-'0'; continue; }
			if (gpsElemType == 10) { gpsH_l = gpsH_l*10 + gpsBuffer[i]-'0'; continue; }
			if (gpsElemType == 11) {
                gpsNavStat_buf[0] = gpsNavStat_buf[1];
                gpsNavStat_buf[1] = gpsBuffer[i];
                continue;
            }
            if (gpsElemType == 12) { hAcc_m = hAcc_m*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 13) { hAcc_l = hAcc_l*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 14) { vAcc_m = vAcc_m*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 15) { vAcc_l = vAcc_l*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 16) {
                if (gpsBuffer[i] == '-') { speed_p = -1; continue; }
                speed_m = speed_m*10 + gpsBuffer[i]-'0'; continue;
            }
            if (gpsElemType == 17) { speed_l = speed_l*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 18) { COG_m = COG_m*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 19) { COG_l = COG_l*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 20) {
                if (gpsBuffer[i] == '-') { speedV_p = -1; continue; }
                speedV_m = speedV_m*10 + gpsBuffer[i]-'0'; continue;
            }
            if (gpsElemType == 21) { speedV_l = speedV_l*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 24) { HDOT_m = HDOT_m*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 25) { HDOT_l = HDOT_l*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 26) { VDOT_m = VDOT_m*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 27) { VDOT_l = VDOT_l*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 28) { TDOT_m = TDOT_m*10 + gpsBuffer[i]-'0'; continue; }
            if (gpsElemType == 29) { TDOT_l = TDOT_l*10 + gpsBuffer[i]-'0'; continue; }
			if (gpsElemType == 30) { gpsStlCount_buf = gpsStlCount_buf*10 + gpsBuffer[i]-'0'; continue; }
		}
	}
}

double addToDuoble(uint32_t m, uint32_t l) {
	double q = l;
	while (q >= 1) { q /= 10; }
	return m + q;
}

float getLat() {
	// gpsLat = GGMM.MM
	int16_t GG__ = gpsLat/100;
	float minutes = gpsLat - GG__*100;
	if (!(gpsVector & 0b10)) {
		return -GG__ - minutes/60;
	}
	return GG__ + minutes/60;
}

float getLon() {
	// gpsLat = GGGMM.MM
	int16_t GGG__ = gpsLon/100;
	float minutes = gpsLon - GGG__*100;
	//return minutes;
	if (!(gpsVector & 0b01)) {
		return -GGG__ - minutes/60;
	}
	return GGG__ + minutes/60;
}

float getTime() { return gpsTime; }

float getAltitude() { return gpsH; }

float getNavigationType() { return gpsNavStat; }
bool if3DFixAvailable() { if (gpsNavStat == 3 || gpsNavStat == 5) { return 1; } return 0; }

float getSpeedHorizontal() { return speed; }
float getSpeedVertical() { return speedV; }

float getAccHorizontal() { return hAcc; }
float getAccVertical() { return vAcc; }

float getCOG() { return COG; }
float getHDOP() { return HDOT; }
float getVDOP() { return VDOT; }
float getTDOP() { return TDOT; }

float getStlCount() { return gpsStlCount; }
}
