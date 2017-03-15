//
// VSON PM050 library for Arduino
// (c)2017, a.m.emelianov@gmail.com
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU License.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU License V3 for more details.
//

#pragma once
#include <Arduino.h>

#define PM050_PIN		     D5
#define PM050_BAUDRATE       9600
#define PM050_MODE			 SERIAL_8N1
#define PM050_FRAME_SIZE     24
#define PM050_FRAME_SIG_SIZE 4
#define PM050_FRAME_SIG    { 0x42, 0x4D, 0x14, 0x00 }
#define PM050_FRAME_PM10H    10
#define PM050_FRAME_PM10L    11
#define PM050_FRAME_PM25H    12
#define PM050_FRAME_PM25L    13
#define PM050_FRAME_PM100H   14
#define PM050_FRAME_PM100L   15
#define PM050_FRAME_CRCH     22
#define PM050_FRAME_CRCL     23
#define PM050_DELAY			 20

template <typename S>
// Possible declarations are
// PM050<HardwareSerial> pm(Serial);
// PM050<SoftwareSerial> pm(SoftSerial);
class PM050 {
public:
	uint16_t pm10	= 0;		// PM 1.0 value
	uint16_t pm25	= 0;		// PM 2.5 value
	uint16_t pm100	= 0;		// PM 10.0 value
	bool read();
	// Read data from sensor and set PMxx
	bool 	 isUpdated();
	// Returns True if PMxx value changed since last isUpdated call
	PM050(S* s, int16_t p = -1);
	// Constructor parameters:
	// - Serial port object reference
	// - GPIO pin connected to sensor's SET pin. Can be omitted. -1 -- not control SET pin.
	void	 begin();
	// Initialize Serial port & start measuring
	void reset();
	// Reset Serial port
	uint8_t noData();
	// Data read try since last correct data packet received
	void measure(bool m);
	// If SET pin used controls Measure/Idle state of sensor
	// Parameter:
	// true - perform measure
	// false - enter idle state
	void startMearure();
	// Start measure
	void stopMearure();
	// Enter sleep state
	bool isMeasure();
	//  If SET pin used returns current Measure state
private:
	S*	 	 _serial;	// Serial port object
	uint8_t	 _pin;		// SET GPIO pin
	bool	 _isUpdated;// Is data updated since last isUpdated() call 
	uint16_t _loops;	// Data read try count
	uint8_t  _pos;		// Current buffer position
	uint8_t  _buf[PM050_FRAME_SIZE];	// Data frame buffer
	uint16_t crc();
	// Calculates Data frame checksum
};

template <typename S> bool PM050<S>::read() {
	uint16_t tPM10;
	uint16_t tPM25;
	uint16_t tPM100;
	uint16_t check;
	uint8_t sig[PM050_FRAME_SIG_SIZE] = PM050_FRAME_SIG;
	_loops++;
	while (_serial->available()) {
		_buf[_pos] = _serial->read();
		if (_pos < PM050_FRAME_SIG_SIZE && _buf[_pos] != sig[_pos]) {
			_serial->print("error");
			_pos = 0;
			continue;
		}
		_pos++;
		if (_pos >= PM050_FRAME_SIZE) {
			_pos = 0;
			check = _buf[PM050_FRAME_CRCH];
			check <<= 8;
			check += _buf[PM050_FRAME_CRCL];
			if (crc() == check) {
				_loops = 0;
				tPM10  =  _buf[PM050_FRAME_PM10H];
				tPM10  <<= 8;
				tPM10  += _buf[PM050_FRAME_PM10L];
				tPM25  += _buf[PM050_FRAME_PM25H];
				tPM25  <<= 8;
				tPM25  += _buf[PM050_FRAME_PM25L];
				tPM100 =  _buf[PM050_FRAME_PM100H];
				tPM100 <<= 8;
				tPM100 += _buf[PM050_FRAME_PM100L];
				if (pm10 != tPM10 || pm25 != tPM25 || pm100 != tPM100) {
					_isUpdated = true;
					pm10  = tPM10;
					pm25  = tPM25;
					pm100 = tPM100;
				}
			}
		}
	}
	return _isUpdated;
}
template <typename S> bool PM050<S>::isUpdated() {
	bool res;
	res	= _isUpdated;
	_isUpdated = false;
	return res;
}
template <typename S> PM050<S>::PM050(S* s, int16_t p) {
	_serial = s;
	_pin = p;
	_pos = 0;
	_isUpdated = false;
	_loops = 0;
}
template <typename S> void PM050<S>::begin() {
	_serial->begin(PM050_BAUDRATE);
	_pos = 0;
	_loops = 0;
	if (_pin > 0) {
		pinMode(_pin, OUTPUT);
		measure(true);
	}
}
template <typename S> void PM050<S>::reset() {
	_serial->end();
	delay(PM050_DELAY);
	begin();
}
template <typename S> uint8_t PM050<S>::noData() {
	return _loops;
}
template <typename S> void PM050<S>::measure(bool m) {
	if (_pin > 0) {
		digitalWrite(_pin, m?LOW:HIGH);
	}
}
template <typename S> void PM050<S>::startMearure() {
	measure(true);
}	
template <typename S> void PM050<S>::stopMearure() {
	measure(false);
}	
template <typename S> bool PM050<S>::isMeasure() {
	return _pin > 0 && digitalRead(_pin) == LOW;
}
template <typename S> uint16_t PM050<S>::crc() {
	uint16_t sum = 0;
	for (uint8_t i = 0; i < PM050_FRAME_SIZE - 2; i++) {
		sum += _buf[i];
	}
	return sum;
}
