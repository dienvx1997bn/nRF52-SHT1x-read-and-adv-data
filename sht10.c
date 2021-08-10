#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"

#include "sht10.h"


#define MSBFIRST	1
#define LSBFIRST	0

#define SCLK_PIN	27
#define SDA_PIN		26
#define HIGH		1
#define LOW			0

uint8_t _dataPin = 26;
uint8_t _clockPin = 27;

/**
 * Reads the current raw temperature value
 */
float sht10_readTemperatureRaw()
{
  int _val;

  // Command to send to the SHT1x to request Temperature
  int _gTempCmd  = 0x03;

  sht10_sendCommandSHT(_gTempCmd, _dataPin, _clockPin);
  sht10_waitForResultSHT(_dataPin);
  _val = sht10_getData16SHT(_dataPin, _clockPin);
  sht10_skipCrcSHT(_dataPin, _clockPin);

  return (_val);
}


/**
 * Reads the current temperature in degrees Celsius
 */
float sht10_readTemperatureC()
{
  int _val;                // Raw value returned from sensor
  float _temperature;      // Temperature derived from raw value

  // Conversion coefficients from SHT15 datasheet
  const float D1 = -40.0;  // for 14 Bit @ 5V
  const float D2 =   0.01; // for 14 Bit DEGC

  // Fetch raw value
  _val = sht10_readTemperatureRaw();

  // Convert raw value to degrees Celsius
  _temperature = (_val * D2) + D1;

  return (_temperature);
}


/**
 * Reads the current temperature in degrees Fahrenheit
 */
float sht10_readTemperatureF()
{
  int _val;                 // Raw value returned from sensor
  float _temperature;       // Temperature derived from raw value

  // Conversion coefficients from SHT15 datasheet
  const float D1 = -40.0;   // for 14 Bit @ 5V
  const float D2 =   0.018; // for 14 Bit DEGF

  // Fetch raw value
  _val = sht10_readTemperatureRaw();

  // Convert raw value to degrees Fahrenheit
  _temperature = (_val * D2) + D1;

  return (_temperature);
}

/**
 * Reads current temperature-corrected relative humidity
 */
float sht10_readHumidity()
{
  int _val;                    // Raw humidity value returned from sensor
  float _linearHumidity;       // Humidity with linear correction applied
  float _correctedHumidity;    // Temperature-corrected humidity
  float _temperature;          // Raw temperature value

  // Conversion coefficients from SHT15 datasheet
  const float C1 = -4.0;       // for 12 Bit
  const float C2 =  0.0405;    // for 12 Bit
  const float C3 = -0.0000028; // for 12 Bit
  const float T1 =  0.01;      // for 14 Bit @ 5V
  const float T2 =  0.00008;   // for 14 Bit @ 5V

  // Command to send to the SHT1x to request humidity
  int _gHumidCmd = 0x05;

  // Fetch the value from the sensor
  sht10_sendCommandSHT(_gHumidCmd, _dataPin, _clockPin);
  sht10_waitForResultSHT(_dataPin);
  _val = sht10_getData16SHT(_dataPin, _clockPin);
  sht10_skipCrcSHT(_dataPin, _clockPin);

  // Apply linear conversion to raw value
  _linearHumidity = C1 + C2 * _val + C3 * _val * _val;

  // Get current temperature for humidity correction
  _temperature = sht10_readTemperatureC();

  // Correct humidity value for current temperature
  _correctedHumidity = (_temperature - 25.0f ) * (T1 + T2 * _val) + _linearHumidity;

  return (_correctedHumidity);
}

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
     uint8_t i;
	nrf_gpio_cfg_output(_dataPin);
	nrf_gpio_cfg_output(_clockPin);
	nrf_gpio_pin_write(_clockPin, LOW);
	nrf_gpio_pin_write(_dataPin, LOW);

     for (i = 0; i < 8; i++)  {
		nrf_gpio_pin_write(clockPin, LOW);

		if (bitOrder == LSBFIRST)
			nrf_gpio_pin_write(dataPin, !!(val & (1 << i)));
		else
			nrf_gpio_pin_write(dataPin, !!(val & (1 << (7 - i))));

		nrf_gpio_pin_write(clockPin, HIGH);

     }
	 nrf_gpio_pin_write(clockPin, LOW);

}

int shiftIn(int _dataPin, int _clockPin, int _numBits)
{
  int ret = 0;
  int i;
	nrf_gpio_cfg_input(_dataPin, NRF_GPIO_PIN_NOPULL);

  for (i=0; i<_numBits; ++i)
  {
     nrf_gpio_pin_write(_clockPin, HIGH);
     nrf_delay_us(10);  // I don't know why I need this, but without it I don't get my 8 lsb of temp
     ret = ret*2 + nrf_gpio_pin_read(_dataPin);
     nrf_gpio_pin_write(_clockPin, LOW);

  }

  return(ret);
}

void sht10_sendCommandSHT(uint8_t _command, uint8_t _dataPin, uint8_t _clockPin)
{
	uint8_t ack;

	// Transmission Start
	nrf_gpio_cfg_output(_dataPin);
	nrf_gpio_cfg_output(_clockPin);

	nrf_gpio_pin_write(_clockPin, LOW);

	nrf_gpio_pin_write(_dataPin, HIGH);

	nrf_gpio_pin_write(_clockPin, HIGH);

	nrf_gpio_pin_write(_dataPin, LOW);

	nrf_gpio_pin_write(_clockPin, LOW);

	nrf_gpio_pin_write(_clockPin, HIGH);

	nrf_gpio_pin_write(_dataPin, HIGH);

	nrf_gpio_pin_write(_clockPin, LOW);


	// The command (3 msb are address and must be 000, and last 5 bits are command)
	shiftOut(_dataPin, _clockPin, MSBFIRST, _command);

	// Verify we get the correct ack
	nrf_gpio_pin_write(_clockPin, HIGH);
	nrf_gpio_cfg_input(_dataPin, NRF_GPIO_PIN_NOPULL);
	ack = nrf_gpio_pin_read(_dataPin);
	if (ack != LOW) {
		//Serial.pruint8_tln("Ack Error 0");
	}
	nrf_gpio_pin_write(_clockPin, LOW);

	ack = nrf_gpio_pin_read(_dataPin);
	if (ack != HIGH) {
		//Serial.pruint8_tln("Ack Error 1");
	}
}

/**
 */
void sht10_waitForResultSHT(uint8_t _dataPin)
{
	uint8_t i;
	uint8_t ack = LOW;

	nrf_gpio_cfg_input(_dataPin, NRF_GPIO_PIN_NOPULL);

	for(i= 0; i < 200; ++i)
	{
		nrf_delay_ms(10);
		ack = nrf_gpio_pin_read(_dataPin);

		if (ack == LOW) {
		  break;
		}
	}

	if (ack == HIGH) {
	//Serial.pruint8_tln("Ack Error 2"); // Can't do serial stuff here, need another way of reporting errors
	}
}

int sht10_getData16SHT(int _dataPin, int _clockPin)
{
  int val;

  // Get the most significant bits
  nrf_gpio_cfg_input(_dataPin, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_output(_clockPin);
  val = shiftIn(_dataPin, _clockPin, 8);
  val *= 256;

  // Send the required ack
  nrf_gpio_cfg_output(_dataPin);
  nrf_gpio_pin_write(_dataPin, HIGH);

  nrf_gpio_pin_write(_dataPin, LOW);

  nrf_gpio_pin_write(_clockPin, HIGH);

  nrf_gpio_pin_write(_clockPin, LOW);


  // Get the least significant bits
  nrf_gpio_cfg_input(_dataPin, NRF_GPIO_PIN_NOPULL);
  val |= shiftIn(_dataPin, _clockPin, 8);

  return val;
}

/**
 */
void sht10_skipCrcSHT(int _dataPin, int _clockPin)
{
  // Skip acknowledge to end trans (no CRC)
  nrf_gpio_cfg_output(_dataPin);
  nrf_gpio_cfg_output(_clockPin);

  nrf_gpio_pin_write(_dataPin, HIGH);

  nrf_gpio_pin_write(_clockPin, HIGH);

  nrf_gpio_pin_write(_clockPin, LOW);

}
