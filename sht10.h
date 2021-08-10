#ifndef SHT10_H
#define SHT10_H

#include <stdint.h>

void sht10_sendCommandSHT(uint8_t _command, uint8_t _dataPin, uint8_t _clockPin);
void sht10_waitForResultSHT(uint8_t _dataPin);
int sht10_getData16SHT(int _dataPin, int _clockPin);
void sht10_skipCrcSHT(int _dataPin, int _clockPin);

float sht10_readTemperatureC(void);
float sht10_readTemperatureF(void);
float sht10_readHumidity(void);
	
#endif
