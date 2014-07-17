// Manylabs BLEShieldStream Arduino Library 0.1.0
// a stream wrapper for the Red Bear Lab BLEShield library
// copyright Manylabs 2014
// MIT license 
#ifndef _BLE_SHIELD_STREAM_H_
#define _BLE_SHIELD_STREAM_H_
#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include <ble_shield.h>


// a stream wrapper for the Red Bear Lab BLEShield library
class BLEShieldStream : public Stream {
public:

	// basic constructor: init member variables
	// fix(soon): allow specifying pins (currently assume that pins 8 and 9 are used for RDYN and REQN)
	BLEShieldStream() {
		_serialDebug = false;
		_initDone = false;
		_checkSum = 0;
	}

	// initialize shield
	inline void begin() {
		ble_begin(); 
		_initDone = true;
	}

	// write a byte
	size_t write( byte data ) {
		_checkSum += data;
		if (_initDone == false)
			begin();
		if (_serialDebug)
			Serial.write( data );
		ble_write( data );
		return 1;
	}

	// read a byte
	inline int read() { 
		if (_initDone == false)
			begin();
		return ble_read();
	}

	// check if data is available to be read
	int available() {
		if (_initDone == false)
			begin();
		return ble_available();
	}

	// not implemented (to do)
	int peek() { return 0; }
	void flush() {}

	// allow the shield to send/receive data
	inline void doEvents() {
		ble_do_events();
	}

	// return true if connected
	inline bool connected() const {
		return ble_connected();
	}

	// reset checksum to zero
	void resetCheckSum() { 
		_checkSum = 0;
	}

	// get current checksum
	inline int checkSum() {
		return _checkSum;
	}

	// set whether or not to display output (sent to BLE shield) on primary serial port
	void setSerialDebug( bool serialDebug ) {
		_serialDebug = true;
	}

private:

	// internal data
	byte _checkSum;
	bool _serialDebug;
	bool _initDone;
};


#endif // _BLE_SHIELD_STREAM_H_
