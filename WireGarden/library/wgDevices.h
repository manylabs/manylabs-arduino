// Manylabs WireGarden Arduino Library
// some code adapted from outside sources
// other code copyright ManyLabs 2011-2014
// dual license:
// (1) MIT license (without external GPL'd libraries)
// (2) GPL license (with external GPL'd libraries)
#ifndef _WIREGARDEN_DEVICES_H_
#define _WIREGARDEN_DEVICES_H_
#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "wgDeviceTypes.h"
#include "wgSend.h"
#include "wgErrorCodes.h"
#include "math.h" // Included by default. Just being explicit.


// device-specific headers
#if defined(USE_ATLAS_SERIAL_DEVICE) || defined(USE_MAXBOTIX_SERIAL_DEVICE) || defined(USE_GPS_DEVICE)
#define USE_SERIAL_DEVICE
#endif
#if defined(USE_GROVE_COLOR_DEVICE) || defined(USE_GROVE_MOTOR_DEVICE) || defined(USE_MINI_PH_DEVICE) || defined(USE_HIH6130_DEVICE)
#include <Wire.h>
#endif
#ifdef USE_DHT_DEVICE
#include <DHT.h>
#endif
#ifdef USE_DS18B20_DEVICE
#include <OneWire.h>
#endif
#ifdef USE_SERVO_DEVICE
#include <Servo.h>
#endif
#ifdef USE_BMP085_DEVICE
#include <Wire.h>
#include <Adafruit_BMP085.h>
#endif
#ifdef USE_BMP180_DEVICE
#include <Wire.h>
#include <Barometer.h>
#endif
#ifdef USE_SERIAL_DEVICE
#include <SoftwareSerial.h>
#endif
#ifdef USE_TSL2561_DEVICE
#include <Wire.h>
#include <TSL2561.h>
#endif
#ifdef USE_HMC5883L_DEVICE
#include <Wire.h>
#include <HMC5883L.h>
#endif
#ifdef USE_ADXL345_DEVICE
#include <Wire.h>
#include <ADXL345.h>
#endif
#ifdef USE_ENCODER_DEVICE
#include <Encoder.h>
#endif
#ifdef USE_INA219_DEVICE
#include <Wire.h>
#include <Adafruit_INA219.h>
#endif
#ifdef USE_RF_DEVICE
#include <VirtualWire.h>
#endif


// The Device class represents a physical hardware device;
// multiple nodes may be get values from a single device;
// we may have multiple device instances of the same type
class Device {
public:

	// initialize device; store pin (can be zero if not needed)
	Device( unsigned char pin ){
		_pin = pin;
		_type = 0;
	}

	// check the state of the sensor object by reading from hardware;
	// for some devices this function must be called frequently in order to obtain sensor values (timing of calls to check may be irregular)
	virtual void check() {}

	// for multi-value devices, this must be called before calling the simple access functions; this is just an alias for calling values() (and ignoring the return values)
	void refresh() { values(); }

	// temporary alias for refresh function (for backward compatibility)
	void update() { values(); }

	// returns number of different kinds of values (e.g. temperature and humidity) provides by the device
	virtual int valueCount() { return 1; }

	// if the device returns a single value, use this to get the value
	virtual float value() { return 0; }

	// if the device returns multiple values, use this to get the value;
	// returns pointer to internval memory; valid for lifetime of device instance;
	// do not attempt to free the returned pointer;
	// will return NULL if and only if valueCount() is less than 2
	virtual float *values() { return 0; }

	// for output devices, set the output value
	virtual void setValue( float value ) {}

	// for output devices with multi-variate outputs, set the output values;
	// assumes values is allocated to contain valueCount elements
	virtual void setValues( float *values ) {}

	// number of decimal places to use when sending data
	virtual int decimalPlaces() { return 2; }

	// get the pin (if any) assocatied with this device
	inline unsigned char pin() const { return _pin; }

	// get the type (if any) assocatied with this device
	inline unsigned char type() const { return _type; }

	// set the type of the device so we can find it later using the type
	inline void setType( unsigned char type ) { _type = type; }

protected:

	// the pin (if any) assocatied with this device
	unsigned char _pin;

	// the type (if any) associated with this device
	unsigned char _type;
};


// define fixed-size data types
typedef unsigned char uint8;
typedef unsigned int uint16;
typedef unsigned long int uint32;


//============================================
// DEVICES
//============================================


// obtains values from an analog input pin
class AnalogInputDevice : public Device {
public:

	AnalogInputDevice( byte pin ) : Device( pin ) {
		_useScaleOffset = 0;
	}

	// remap the input values so voltages range from offset to offset + scaleFactor
	AnalogInputDevice( byte pin, float offset, float scaleFactor ) : Device( pin ) {
		_offset = offset;
		_scaleFactor = scaleFactor / 1023.0f;
		_useScaleOffset = 1;
	}

	// optional args: offset and scale factor;
	// remaps the input values so voltages range from offset to offset + scaleFactor
	AnalogInputDevice( byte pin, char *args[], int argCount ) : Device( pin ) {
		if (argCount == 2) {
			_offset = atof( args[ 0 ] );
			_scaleFactor = atof( args[ 1 ] ) / 1023.0f;
			_useScaleOffset = 1;
		} else {
			_useScaleOffset = 0;
		}
	}

	float value() {
		float v = (float) analogRead( _pin );
		if (_useScaleOffset) {
			v = v * _scaleFactor + _offset;
		}
		return v;
	}

private:

	// internal data
	unsigned char _useScaleOffset;
	float _scaleFactor;
	float _offset;
};


// obtains a value from a digital input pin
class BinaryInputDevice : public Device {
public:

	BinaryInputDevice( byte pin ) : Device( pin ) {
		pinMode( pin, INPUT );
	}

	BinaryInputDevice( byte pin, char *args[], int argCount ) : Device( pin ) {
		boolean usePullup = false;
		// If the first additional parameter is 1 then the pin is an analog pin we're
		// treating as digital
		if (argCount >= 1) {
			if(atoi(args[0])){
				_pin = analogInputToDigitalPin( pin );
			}
		}
		// If the second additional parameter is true, then this will be se as INPUT_PULLUP
		// instead of just INPUT
		if (argCount >= 2) {
			if(atoi(args[1])){
				usePullup = true;
			}
		}
		if(usePullup){
			pinMode(_pin, INPUT_PULLUP);
		}else{
			pinMode(_pin, INPUT);
		}
	}

	float value() {
		return (float) digitalRead( _pin );
	}

	// number of decimal places to use when sending data
	int decimalPlaces() { return 0; };
};


// sets the value of a digital output pin
class BinaryOutputDevice : public Device {
public:

	BinaryOutputDevice( byte pin ) : Device( pin ) {
		pinMode( pin, OUTPUT );
	}

	BinaryOutputDevice( byte pin, char *args[], int argCount ) : Device( pin ) {
		// If the first additional parameter is 1 then the pin is an analog pin we're
		// treating as digital
		if (argCount >= 1) {
			if(atoi(args[0])){
				_pin = analogInputToDigitalPin( pin );
			}
		}
		pinMode(_pin, OUTPUT);
	}

	void setValue( float value ) {
		int valueInt = (int) value;
		if (valueInt < 0) valueInt = 0;
		if (valueInt > 1) valueInt = 1;
		digitalWrite( _pin, valueInt );
	}
};


// sets the value (duty cycle) of a pulse-width-modulated (PWM) output pin
class PulseWidthDevice : public Device {
public:

	PulseWidthDevice( byte pin ) : Device( pin ) {
		pinMode( pin, OUTPUT );
	}

	// set duty cycle; value between 0 (always off) and 255 (always on)
	void setValue( float value ) {
		int valueInt = (int) value;
		if (valueInt < 0) valueInt = 0;
		if (valueInt > 255) valueInt = 255;
		analogWrite( _pin, valueInt );
	}
};


// You can't use the tone device with the pulse sensor device. They both use timer2
#ifndef USE_PULSE_SENSOR_DEVICE
// generate a tone on a digital output pin
class ToneDevice : public Device {
public:

	ToneDevice( byte pin ) : Device( pin ) {
		pinMode( pin, OUTPUT );
	}

	void setValue( float value ) {
		int valueInt = (int) value;
		if (valueInt)
			tone( _pin, valueInt );
		else
			noTone( _pin );
	}
};
#endif


// This code is designed to work with the standard Grove analog temperature sensor; you can use it with other similar sensors by
// passing in beta, resistanceRoomTemp, and seriesResistor values.
class AnalogTemperatureDevice : public Device {
public:

	AnalogTemperatureDevice( byte pin ) : Device( pin ) {
		_B = 3975;
		_R0 = 10000;
		_seriesResistor = 10000;
	}

	AnalogTemperatureDevice( byte pin, float beta, float resistanceRoomTemp, float seriesResistor ) : Device( pin ) {
		_B = beta;
		_R0 = resistanceRoomTemp;
		_seriesResistor = seriesResistor;
	}

	// extra args: beta, resistanceRoomTemp, and seriesResistor
	AnalogTemperatureDevice( byte pin, char *args[], int argCount ) : Device( pin ) {
		_B = 3975;
		_R0 = 10000;
		_seriesResistor = 10000;
		if (argCount >= 3) {
			_B = atof( args[ 0 ] );
			_R0 = atof( args[ 1 ] );
			_seriesResistor = atof( args[ 2 ] );
		}
	}

	int valueCount() { return 3; };

	// most recent sensor value (C, F, resistance in ohms)
	float *values() {
		float analogValue = (float) analogRead( _pin );
		float resistance = (float)(1023 - analogValue) * _seriesResistor / analogValue;
  		float tempC = 1 / ( log(resistance / _R0) / _B + 1 / 298.15 ) - 273.15;
		_values[0] = tempC;
		_values[1] = tempC * 9/5 + 32;
		_values[2] = resistance;
		return _values;
	}

	inline float tempC() const { return _values[ 0 ]; }
	inline float tempF() const { return _values[ 1 ]; }

private:

	// most recent sensor value (C, F, resistance in ohms)
	float _values[ 3 ];

	// internal data
	float _B;
	float _R0;
	float _seriesResistor;
};


//============================================
// SERIAL DEVICE
//============================================


// a special super-class for devices that use serial (handles switching between hardware and software serial)
#ifdef USE_SERIAL_DEVICE
class SerialDevice : public Device {
public:

	// creates a serial port using pin and pin + 1 with the specified baud rate (data transfer rate)
	SerialDevice( byte pin, long baud ) : Device( pin ), _serial( pin, pin + 1 ) {
		#ifdef __AVR_ATmega2560__
		switch (_pin) {
		case 18: Serial1.begin( baud ); break;
		case 16: Serial2.begin( baud ); break;
		case 14: Serial3.begin( baud ); break;
		default: _serial.begin( baud );
		}
		#else
		_serial.begin( baud );
		#endif
	}

	void serialWrite( char *str ) {
		#ifdef __AVR_ATmega2560__
		switch (_pin) {
		case 18: Serial1.write( str ); break;
		case 16: Serial2.write( str ); break;
		case 14: Serial3.write( str ); break;
		default:
			_serial.listen(); // needed to have more than one software serial device at a time
			_serial.write( str );
		}
		#else
		_serial.listen(); // needed to have more than one software serial device at a time
		_serial.write( str );
		#endif
	}

	int serialRead(){
		int v = 0;
		#ifdef __AVR_ATmega2560__
		switch (_pin) {
		case 18: v = Serial1.read(); break;
		case 16: v = Serial2.read(); break;
		case 14: v = Serial3.read(); break;
		default: v = _serial.read();
		}
		#else
		v = _serial.read();
		#endif
		return v;
	}

private:

	SoftwareSerial _serial;
};
#endif // USE_SERIAL_DEVICE


//============================================
// R/C SERVO DEVICE
//============================================


// R/C servo device
#ifdef USE_SERVO_DEVICE
class ServoDevice : public Device {
public:

	// initialize connection with sensor
	ServoDevice( byte pin ) : Device( pin ) {
		_servo.attach( pin );
	}

	// send value to servo
	void setValue( float value ) {
		_servo.write( (int) value );
	}

private:

	// a servo object from the standard Arduino servo library
	Servo _servo;
};
#endif // USE_SERVO_DEVICE


//============================================
// DALLAS TEMPERATURE SENSOR
//============================================


// DS18B20 temperature sensors
#ifdef USE_DS18B20_DEVICE
class DSDevice : public Device {
public:

	// initialize connection with sensor
	DSDevice( byte pin ) : Device( pin ), _oneWire( pin ) {
		_initDone = false;

		// get address
		if (!_oneWire.search( _addr )) {
			_oneWire.reset_search(); // no more sensors on chain, reset search
			return;
		}

		// verify address
		// if (OneWire::crc8( _addr, 7 ) != _addr[ 7 ]) {
		// 	sendError( MEC_DS18B20_BAD_CRC );
		// 	return;
		// }
		if (_addr[ 0 ] != 0x10 && _addr[ 0 ] != 0x28) {
			sendError( MEC_DS18B20_BAD_DEVICE );
			return;
		}
		_initDone = true;
		_valid = false;
	}

	// read data from temperature sensor
	float value() {

		// make sure initialized
		if (_initDone == false)
			return NAN;

		// send commands to get temperature
		_oneWire.reset();
		_oneWire.select( _addr );
		_oneWire.write( 0x44, 1 ); // start conversion, with parasite power on at the end
		_oneWire.reset();
		_oneWire.select( _addr );
		_oneWire.write( 0xBE ); // read scratchpad

		// read data
		byte data[ 9 ];
		for (int i = 0; i < 9; i++) {
			data[ i ] = _oneWire.read();
		}

		// convert data to temperature
		int tempInt = ((data[ 1 ] << 8) | data[ 0 ]);
		if (tempInt == 1360 && _valid == false) // prevent spirious 85.0 degree values on startup by discarding them until we see a different value
			return NAN;
		_valid = true;
		return (float) tempInt / 16.0;
	}

private:

	// internal data
	OneWire _oneWire;
	boolean _initDone;
	boolean _valid;
	byte _addr[ 8 ];
};
#endif // USE_DS18B20_DEVICE


//============================================
// ATLAS SCIENTIFIC TEMPERATURE SENSOR
//============================================


class AtlasTemperatureDevice : public Device {
public:

	// initialize atlas temperature sensor; assumes connected to analog 0 and analog 1
	AtlasTemperatureDevice() : Device( 0 ) {
		pinMode( A1, OUTPUT );
	}

	float value() {
		digitalWrite( A0, LOW ); // for now we assume analog 0, analog 1
		digitalWrite( A1, HIGH );
		delay( 2 );
		float vOut = analogRead( 0 );
		digitalWrite( A1, LOW );
		vOut *= .0048;
		vOut *= 1000;
		return 0.0512 * vOut - 20.5128;
	}
};


//============================================
// MAX6675 THERMOCOUPLE READER
//============================================


// MAX6675 thermocouple amplifier
class MAX6675Device : public Device {
public:

	// initialize connection with sensor
	MAX6675Device( byte pin ) : Device( pin ) {
		pinMode( _pin, INPUT ); // D0
		pinMode( _pin + 1, OUTPUT ); // CS
		pinMode( _pin + 2, OUTPUT ); // CLK
		digitalWrite( _pin + 2, HIGH ); // set CLK high
		_lastReadTimestamp = millis(); // don't read right away; give MAX6675 time to settle
	}

	// read data from thermocouple
	float value() {

		// throttle the reads
		long timestamp = millis();
		if (timestamp - _lastReadTimestamp < 500)
			return NAN;
		_lastReadTimestamp = timestamp;

		// set CS low
		digitalWrite( _pin + 1, LOW );
		delay( 1 ); // 1 ms

		// read 2 bytes
		uint16_t v = spiRead();
		v <<= 8;
		v |= spiRead();

		// set CS high
		digitalWrite( _pin + 1, HIGH);

		// check for no thermocouple connected
		if (v & 0x4) {
			sendError( MEC_NO_THERMOCOUPLE );
			return NAN;
		}

		// get temperature in celsius
		v >>= 3;
		return v * 0.25f;
	}

private:

	// read a byte via SPI
	byte spiRead() {
		byte d = 0;
		for (int i = 7; i >= 0; i--) {
			digitalWrite( _pin + 2, LOW ); // set CLK low
			delay( 1 ); // 1 ms
			if (digitalRead( _pin )) { // read D0
				d |= (1 << i);
			}
			digitalWrite( _pin + 2, HIGH); // set CLK high
			delay( 1 ); // 1 ms
		}
		return d;
	}

	// millisecond timestamp of last sensor reading
	unsigned long _lastReadTimestamp;
};


//============================================
// TSL2561 LIGHT SENSOR
//============================================


// TSL2561 light sensor
#ifdef USE_TSL2561_DEVICE
class TSLDevice : public Device {
public:

	// initialize light sensor; if highGain is true, uses 16X gain;
	// if longIntegration is true, uses 101 msec integration time (otherwise 13 msec)
	TSLDevice( byte highGain = 0, byte longIntegration = 0 ) : Device( 0 ) {
		_tsl = NULL;
		_initDone = 0;
		_highGain = highGain;
		_longIntegration = longIntegration;
	}

	// initialize light sensor; does not take pin since I2C;
	// optional arguments: highGain and longIntegration; if highGain is true, uses 16X gain;
	// if longIntegration is true, uses 101 msec integration time (otherwise 13 msec)
	TSLDevice( char *args[], int argCount ) : Device( 0 ) {
		_tsl = NULL;
		_initDone = 0;
		_highGain = argCount >= 1 && atoi( args[ 0 ] ) > 0;
		_longIntegration = argCount >= 2 && atoi( args[ 1 ] ) > 0;
	}

	~TSLDevice() {
		delete _tsl;
	}

	// four values: lux, full spectrum luminosity, ir luminosity, visible luminosity
	int valueCount() { return 4; }

	// get lux and luminosity
	float *values(){
		uint32_t lux = 0;
		uint16_t visible = 0;
		uint16_t full = 0;
		uint16_t ir = 0;
		if (_tsl == NULL) {
			init();
		}
		if (_initDone) {
			// Taken from TSL2561.cpp
			uint32_t x = _tsl->getFullLuminosity();
			full = x & 0xFFFF;
			ir = x >> 16;
			visible = full - ir;

			lux = _tsl->calculateLux(full, ir);
		}
		_values[0] = (float) lux;
		_values[1] = (float) full;
		_values[2] = (float) ir;
		_values[3] = (float) visible;
		return _values;
	}

	// simple accessors; assumes called refresh()
	inline float lux() const { return _values[ 0 ]; }
	inline float luminosity() const { return _values[ 1 ]; }
	inline float irLuminosity() const { return _values[ 2 ]; }
	inline float visibleLuminosity() const { return _values[ 3 ]; }

private:

	// create and configure TSL device
	void init() {
		_tsl = new TSL2561( TSL2561_ADDR_LOW ); // would be nice not to have to use new, but doesn't work otherwise
		if (_tsl->begin()) {
			_initDone = true;
		} else {
			_initDone = false;
			sendError( MEC_TSL2561_NOT_FOUND );
		}
		_tsl->setGain( _highGain ? TSL2561_GAIN_16X : TSL2561_GAIN_0X );
		_tsl->setTiming( _longIntegration ? TSL2561_INTEGRATIONTIME_101MS : TSL2561_INTEGRATIONTIME_13MS );
	}

	// internal data
	TSL2561 *_tsl;
	boolean _initDone;
	byte _highGain;
	byte _longIntegration;

	// most recent sensor value (lux, full luminosity, ir luminosity, visible luminosity)
	float _values[ 4 ];
};
#endif


//============================================
// ATLAS SCIENTIFIC SERIAL SENSOR
//============================================


// Atlas Scientific serial sensor
#ifdef USE_ATLAS_SERIAL_DEVICE
class AtlasSerialDevice : public SerialDevice {
public:

	AtlasSerialDevice( byte pin ) : SerialDevice( pin, 38400 ) {}

	virtual float value(){
		char buf[ 30 ];
		serialWrite( "R\r" );
		readResponse( buf, 30 );
		float val = NAN;
		if (buf[ 0 ]) {
			val = atof( buf );
		}
		return val;
	}

protected:

	// wait for response from atlas device; read into buffer; timeout after 2 seconds
	void readResponse( char *buf, int bufLen ){
		buf[ 0 ] = 0;
		int pos = 0;
		long time = millis();
		while (millis() - time < 2000 && pos < 30) {
			char c = (char) serialRead();
			if (c == '\r') {
				break;
			}
			if (c > 32) {
				buf[ pos++ ] = c;
			}
		}
		if (pos) {
			buf[ pos ] = 0;
		}
	}
};
#endif // USE_ATLAS_SERIAL_DEVICE


// Atlas Scientific EC (electrical conductivity) serial sensor
#ifdef USE_ATLAS_SERIAL_DEVICE
class AtlasECDevice : public AtlasSerialDevice {
public:

	AtlasECDevice( byte pin ) : AtlasSerialDevice( pin ) {
		_temperature = 20.0f;
	}

	// returns conductivity in microsiemens; can also obtain salinity by calling salinity()
	float value() {

		// request value based on current temperature
		// note: don't need to send a command prefix, just the value
		char convBuf[ 10 ];
		dtostrf( _temperature, 2, 2, convBuf ); // convert temperature to string with 2 places after decimal
		serialWrite( convBuf );
		serialWrite( "\r" );

		// read and process the response;
		// response has three components: conductivity (microsiemens), TDS (PPM), salinity (PSS-78)
		char buf[ 30 ];
		readResponse( buf, 30 );
		float val = NAN;
		if (buf[ 0 ]) {
			val = atof( buf ); // first component is conductivity
			byte compIndex = 0;
			for (byte i = 0; buf[ i ] > 0; i++) { // loop through the reply to find the third component (compIndex == 2)
				if (buf[ i ] == ',') {
					compIndex++;
				}
				if (compIndex == 2) {
					_salinity = atof( buf + i + 1 );
					break;
				}
			}
		}
		return val;
	}

	// set the temperature used by the device in degrees C
	inline void setTemperature( float temperature ) { _temperature = temperature; }

	// returns salinity (using PSS-78); will be zero if out of bounds
	inline float salinity() const { return _salinity; }

private:
	float _temperature; // degrees C (sent to device)
	float _salinity; // salinity (PSS-78); will be zero if out of bounds
};
#endif // USE_ATLAS_SERIAL_DEVICE


// Atlas Scientific DO (dissolved oxygen) serial sensor
#ifdef USE_ATLAS_SERIAL_DEVICE
class AtlasDODevice : public AtlasSerialDevice {
public:

	AtlasDODevice( byte pin ) : AtlasSerialDevice( pin ) {
		_temperature = 20.0f;
		_conductivity = 0;
	}

	float value(){

		// request value based on current temperature and conductivity
		// note: don't need to send a command prefix, just the values
		char convBuf[ 10 ];
		dtostrf( _temperature, 2, 2, convBuf ); // convert temperature to string with 2 places after decimal
		serialWrite( convBuf );
		serialWrite( "," );
		ltoa( _conductivity, convBuf, 10 );
		serialWrite( convBuf );
		serialWrite( "\r" );

		// read and process the response
		char buf[ 30 ];
		readResponse( buf, 30 );
		float val = NAN;
		if (buf[ 0 ]) {
			val = atof( buf );
		}
		return val;
	}
	inline void setTemperature( float temperature ) { _temperature = temperature; } // degrees C
	inline void setConductivity( long conductivity ) { _conductivity = conductivity; } // microsiemens
private:
	float _temperature; // degrees C (sent to device)
	long _conductivity; // microsiemens (sent to device)
};
#endif // USE_ATLAS_SERIAL_DEVICE


//============================================
// ATLAS SCIENTIFIC SERIAL COLOR SENSOR
//============================================


#ifdef USE_ATLAS_SERIAL_DEVICE
class AtlasSerialColorDevice : public SerialDevice {
public:

	// initialize connection with sensor
	AtlasSerialColorDevice( byte pin ) : SerialDevice( pin, 38400 ) {
		_values[ 0 ] = 0;
		_values[ 1 ] = 0;
		_values[ 2 ] = 0;
		_values[ 3 ] = 0;
	}

	// red, green, blue, brightness
	int valueCount() { return 4; };

	// returns red, green, blue, brightness
	float *values() {
		serialWrite( "R\r" );
		char buf[ 30 ];
		int pos = 0;
		long time = millis();
		while (millis() - time < 1000 && pos < 30) { // wait one second, since this device takes 620 ms to reply
			char c = (char) serialRead();
			if (c == '\r') {
				break;
			}
			if (c > 32) {
				buf[ pos++ ] = c;
			}
		}
		if (pos) {
			buf[ pos ] = 0;
			byte compIndex = 0;
			char *compBuf = buf; // pointer to component string within buffer; start with first component
			for (byte i = 0; i <  pos; i++) {
				if (buf[ i ] == ',') {
					compIndex++;
					compBuf = buf + i + 1;
				}
				if (compBuf) {
					int val = atoi( compBuf );
					_values[ compIndex ] = (float) val;
					compBuf = NULL;
				}
			}
		}
		return _values;
	}

	// simple accessors; assumes called refresh()
	inline float red() const { return _values[ 0 ]; }
	inline float green() const { return _values[ 1 ]; }
	inline float blue() const { return _values[ 2 ]; }
	inline float brightness() const { return _values[ 3 ]; }

private:

	// red, green, blue, brightness
	float _values[ 4 ];
};
#endif // USE_ATLAS_SERIAL_DEVICE


//============================================
// GROVE COLOR SENSOR
//============================================


#ifdef USE_GROVE_COLOR_DEVICE
class GroveColorDevice : public Device {
public:

	// initialize connection with sensor; does not take pin since I2C
	// optional arguments: gain, prescale, integration;
	// gain options: 0 -> 1x, 1 -> 4x, 2 -> 16x, 3 -> 64x
	// prescale options: 0 -> 1, 1 -> 2, 2 -> 4, 3 -> 8, 4 -> 16, 5 -> 32, 6 -> 64
	// integration options: 0 -> 12 ms, 1 -> 100 ms, 2 -> 400ms
	GroveColorDevice( int gain = 0, int prescale = 256, int integration = 0 ) : Device( 0 ) {
		_initDone = false;
		_extraScale = false;
		_gain = gain;
		_prescale = prescale;
		if(_prescale == 256){
			_prescale = 6;
			_extraScale = true;
		}
		_integration = integration;
	}

	// initialize connection with sensor; does not take pin since I2C;
	// optional arguments: gain, prescale, integration;
	// gain options: 0 -> 1x, 1 -> 4x, 2 -> 16x, 3 -> 64x;
	// prescale options: 0 -> 1, 1 -> 2, 2 -> 4, 3 -> 8, 4 -> 16, 5 -> 32, 6 -> 64;
	// integration options: 0 -> 12 ms, 1 -> 100 ms, 2 -> 400ms
	GroveColorDevice( char *args[], int argCount  ) : Device( 0 ) {
		_initDone = false;
		_extraScale = false;
		_gain = 0;
		_prescale = 256;
		_integration = 0;
		if (argCount >= 1) {
			_gain = atoi( args[ 0 ] );
		}
		if (argCount >= 2) {
			_prescale = atoi( args[ 1 ] );
		}
		if (argCount >= 3) {
			_integration = atoi( args[ 2 ] );
		}
		if(_prescale == 256){
			_prescale = 6;
			_extraScale = true;
		}
	}

	// red, green, blue, brightness
	int valueCount() { return 4; };

	// returns red, green, blue, brightness
	float *values() {

		if (_initDone == false)
			init();

		// compute and return colors
		unsigned int greenVal = (float) requestGreen();
		unsigned int redVal = (float) requestRed();
		unsigned int blueVal = (float) requestBlue();
		unsigned int brightVal = (float) requestClear();

		// Scale to 256
		if(_extraScale){
			greenVal = greenVal / 4;
			redVal = redVal / 4;
			blueVal = blueVal / 4;
			brightVal = brightVal / 4;
		}
		_values[ 0 ] = (float)(redVal);
		_values[ 1 ] = (float)(greenVal);
		_values[ 2 ] = (float)(blueVal);
		_values[ 3 ] = (float)(brightVal);
		return _values;
	}

	// simple accessors; assumes called refresh()
	inline float red() const { return _values[ 0 ]; }
	inline float green() const { return _values[ 1 ]; }
	inline float blue() const { return _values[ 2 ]; }
	inline float brightness() const { return _values[ 3 ]; }

private:

	// see above for allowed gain/prescale/integration values
	void init() {
		Wire.begin();
		delay( 10 );

		// set gain and prescale
		Wire.beginTransmission( _ADDRESS );
		Wire.write( 0x87 );
		Wire.write( (_gain << 4) | _prescale );
		Wire.endTransmission();

		// set integration time
		Wire.beginTransmission( _ADDRESS );
		Wire.write( 0x81 );
		Wire.write( _integration );
		Wire.endTransmission();

		// Power up
		Wire.beginTransmission( _ADDRESS );
		Wire.write( _CONTROL_REGISTER );
		Wire.write( _POWER_UP );
		Wire.endTransmission();

		// Minimum of 12ms so we'll give it some space
		delay(50);
		_initDone = true;
	}

	unsigned int requestClear() {
		unsigned int clearLow = 0;
		unsigned int clearHigh = 0;
		Wire.beginTransmission(0x39); //0011 1001 //+0 for write
		Wire.write(0xB6); //1011 0110 //read Clear register
		Wire.endTransmission();

		Wire.beginTransmission(0x39); //Request information
		Wire.requestFrom(0x39,2); //0011 1001 //+1 for read
		clearLow = Wire.read();
		clearHigh = Wire.read();
		Wire.endTransmission();

		clearHigh = (clearHigh * 256) + clearLow;
		return clearHigh;
	}

	unsigned int requestRed() {
		unsigned int redLow = 0;
		unsigned int redHigh = 0;
		Wire.beginTransmission(0x39); //0011 1001 //+0 for write
		Wire.write(0xB2); //1011 0010 //read Red register
		Wire.endTransmission();

		Wire.beginTransmission(0x39); //Request information
		Wire.requestFrom(0x39,2); //0011 1001 //+1 for read
		redLow = Wire.read();
		redHigh = Wire.read();
		Wire.endTransmission();

		redHigh = (redHigh * 256) + redLow;
		return redHigh;
	}

	unsigned int requestGreen() {
		unsigned int greenLow = 0;
		unsigned int greenHigh = 0;
		Wire.beginTransmission(0x39); //0011 1001 //+0 for write
		Wire.write(0xB0); //1011 0000 //read Green register
		Wire.endTransmission();

		Wire.beginTransmission(0x39); //Request information
		Wire.requestFrom(0x39,2); //0011 1001 //+1 for read
		greenLow = Wire.read();
		greenHigh = Wire.read();
		Wire.endTransmission();

		greenHigh = (greenHigh * 256) + greenLow;
		return greenHigh;
	}

	unsigned int requestBlue() {
		unsigned int blueLow = 0;
		unsigned int blueHigh = 0;
		Wire.beginTransmission(0x39); //0011 1001 //+0 for write
		Wire.write(0xB4); //1011 0100 //read Blue register
		Wire.endTransmission();

		Wire.beginTransmission(0x39); //Request information
		Wire.requestFrom(0x39,2); //0011 1001 //+1 for read
		blueLow = Wire.read();
		blueHigh = Wire.read();
		Wire.endTransmission();

		blueHigh = (blueHigh * 256) + blueLow;
		return blueHigh;
	}

	// device-specific constants
	static const unsigned char _ADDRESS = 0x39;
	static const unsigned char _CONTROL_REGISTER = 0x80;
	static const unsigned char _TIMING_REGISTER = 0x81;
	static const unsigned char _BLOCKREAD_REGISTER = 0xCF;
	static const unsigned char _POWER_UP = 0x03;

	// red, green, blue, brightness
	float _values[ 4 ];

	// sensor configuration (set by constructor and used in init)
	int _gain;
	int _prescale;
	int _integration;

	// var for extra scaling
	bool _extraScale;

	// true done with call to init()
	boolean _initDone;
};
#endif // USE_GROVE_COLOR_DEVICE


//============================================
// GROVE I2C MOTOR DRIVER
//============================================


#ifdef USE_GROVE_MOTOR_DEVICE
class MotorDriverDevice : public Device {
public:

	// does not take pin since I2C
	MotorDriverDevice() : Device( 0 ) {
		_m1Direction = -1;
		_m2Direction = -1;
		_m1Speed = -1;
		_m2Speed = -1;
		Wire.begin();
		delayMicroseconds(10); // MotorDriver init
	}

	// set motor speed and direction for two motors:
	// values[ 0 ] = motor 1 direction
	// values[ 1 ] = motor 1 speed
	// values[ 2 ] = motor 1 direction
	// values[ 3 ] = motor 2 speed
	void setValues( float *values ){

		// Get values for Speed and Direction
		uint8 m1Direction = (int) (values[ 0 ] + 0.5f); // round to nearest (assuming positive)
		uint8 m1Speed = (int) (values[ 1 ] + 0.5f);
		uint8 m2Direction = (int) (values[ 2 ] + 0.5f);
		uint8 m2Speed = (int) (values[ 3 ] + 0.5f);

		if(m1Direction != _m1Direction || m1Speed != _m1Speed || m2Direction != _m2Direction || m2Speed != _m2Speed){
			_m1Direction = m1Direction;
			_m1Speed = m1Speed;
			_m2Direction = m2Direction;
			_m2Speed = m2Speed;
			setBothMotors(m1Direction, m1Speed, m2Direction, m2Speed);
		}
	}

	// set motor speed and direction for two motors
	void setBothMotors( int m1Dir, int m1Spd, int m2Dir, int m2Spd ){

		// validation
		if(m1Dir > 3)
			m1Dir = 3;
		if(m1Dir < 0)
			m1Dir = 0;
		if(m2Dir > 3)
			m2Dir = 3;
		if(m2Dir < 0)
			m2Dir = 0;
		if(m1Spd > 255)
			m1Spd = 255;
		if(m1Spd < 0)
			m1Spd = 0;
		if(m2Spd > 255)
			m2Spd = 255;
		if(m2Spd < 0)
			m2Spd = 0;

		// set direction
		Wire.beginTransmission(_MOTORSHIELD_ADDR); // transmit to device MOTORSHIELDaddr
		Wire.write(_CHANNELSET);        // channel control header
		Wire.write(4*m2Dir + m1Dir);                // send channel control information
		Wire.write((uint8_t)0);               // need to send this byte as the third byte(no meaning)
		Wire.endTransmission();

		// set speed
		Wire.beginTransmission(_MOTORSHIELD_ADDR); // transmit to device MOTORSHIELDaddr
		Wire.write(_SETPWMAB);        //set pwm header
		Wire.write(m1Spd);              // send pwma
		Wire.write(m2Spd);              // send pwmb
		Wire.endTransmission();      // stop transmitting
	}

	void setSingleMotor( unsigned char motor, int direction, int speed ){
		unsigned char selectedMotor = _m1;
		unsigned char selectedDirection = _FORWARD;
		unsigned char selectedSpeed = 0;

		// validation
		if(motor == _m2){
			selectedMotor = _m2;
		}
		if(direction == 1){
			selectedDirection = _REVERSE;
		}
		if(speed >= 0 && speed <= 255){
			selectedSpeed = speed;
		}

		// send commands
		Wire.beginTransmission(_MOTORSHIELD_ADDR);
		Wire.write(selectedMotor);
		Wire.write(selectedDirection);
		Wire.write(selectedSpeed);
		Wire.endTransmission();
	}

private:
	static const unsigned char _MOTORSHIELD_ADDR = 0x28;
	static const unsigned char _CHANNELSET = 0xaa;
	static const unsigned char _SETPWMAB = 0x82;
	static const unsigned char _m1 = 0xA1;
	static const unsigned char _m2 = 0xA5;
	static const unsigned char _FORWARD = 0B01;
	static const unsigned char _REVERSE = 0B10;
	byte _m1Direction;
	byte _m2Direction;
	byte _m1Speed;
	byte _m2Speed;
};
#endif // USE_GROVE_MOTOR_DEVICE


//============================================
// DHT TEMPERATURE / HUMIDITY SENSOR
//============================================


// DHT22 and similar temperature / humidity sensor
#ifdef USE_DHT_DEVICE
class DHTDevice : public Device {
public:

	DHTDevice( byte pin, byte type ) : Device( pin ), _dht( pin, libType( type ) ) {}

	// two values: temperature and humidity
	int valueCount() { return 2; }

	// returns temperature (degrees C) and humidity
	float *values() {
		float temperature = _dht.readTemperature();
		float humidity = _dht.readHumidity();
		if (!isnan( temperature ))
			_values[ 0 ] = temperature;
		if (!isnan( humidity ))
			_values[ 1 ] = humidity;
		return _values;
	}

	// last temperature value (degrees C); assumes called refresh()
	inline float temperature() const { return _values[ 0 ]; }

	// last temperature value (degrees C); assumes called refresh()
	inline float humidity() const { return _values[ 1 ]; }

private:

	// convert WireGarden node type to DHT library device type
	static byte libType( byte type ){
		switch (type) {
		case DEVICE_TYPE_DHT11: return DHT11;
		case DEVICE_TYPE_DHT21: return DHT21;
		case DEVICE_TYPE_DHT22: return DHT22;
		case DEVICE_TYPE_AM2301: return AM2301;
		}
		return 0;
	}

	DHT _dht;
	float _values[ 2 ];
};
#endif // USE_DHT_DEVICE


//============================================
// HIH6130 TEMPERATURE / HUMIDITY SENSOR
//============================================


#ifdef USE_HIH6130_DEVICE
class HIH6130Device : public Device {
public:

	//does not take pin since I2C
	HIH6130Device() : Device( 0 ){
		_initDone = false;
	}

	// two values: temperature and humidity
	int valueCount() { return 2; }

	// returns temperature (degrees C) and humidity
	float *values() {
		if (_initDone == false)
			init();

		float temperature;
		float humidity;
		uint8_t status = readTemperatureAndHumidity(temperature, humidity);
		if(status == 0){
			_values[ 0 ] = temperature;
			_values[ 1 ] = humidity;
		}
		return _values;
	}

	// last temperature value (degrees C); assumes called refresh()
	inline float temperature() const { return _values[ 0 ]; }

	// last humidity value (percent); assumes called refresh()
	inline float humidity() const { return _values[ 1 ]; }

private:

	float _values[ 2 ];
	bool _initDone;

	void init(){
		Wire.begin();
		_initDone = true;
	}

	// Takes a float for temperature and humidity and fills them with the current
	// values.
	// Returns a status code:
	// 0: Normal
	// 1: Stale - this data has already been read
	// 2: Command Mode - the sensor is in command mode
	// 3: Diagnostic - The sensor has had a diagnostic condition and data is
	//    invalid
	// 4: Invalid - The sensor did not return 4 bytes of data. Usually this means
	//    it's not attached.
	uint8_t readTemperatureAndHumidity( float &temperature_c, float &humidity_percent ) {
		// From: http://www.phanderson.com/arduino/hih6130.html
		uint8_t address = 0x27;
		uint8_t humHigh, humLow, tempHigh, tempLow, status;
		uint16_t humData, tempData;

		// Request read
		Wire.beginTransmission(address);
		Wire.endTransmission();

		// According to the data sheet, the measurement cycle is typically ~36.56 ms
		// We'll give a little extra time
		delay(50);

		// Request data
		uint8_t bytesReceived = Wire.requestFrom((int)address, (int)4);
		if(bytesReceived != 4){

			// This is our own error to specify that we didn't receive 4 bytes from the
			// sensor.
			return 4; // temp and humidity will be unchanged
		}

		humHigh  = Wire.read();
		humLow   = Wire.read();
		tempHigh = Wire.read();
		tempLow  = Wire.read();

		// Status is the top two bits of the high humidity byte
		status = (humHigh >> 6) & 0x03;
		if(status == 3){
			return status; // temp and humidity will be unchanged
		}

		// Keep the rest
		humHigh = humHigh & 0x3f;

		// OR in the low bytes
		humData  = (((uint16_t)humHigh)  << 8) | humLow;
		tempData = (((uint16_t)tempHigh) << 8) | tempLow;

		// The bottom two bits of the low temp byte are invalid, so we'll remove
		// those
		tempData = tempData >> 2;

		// Convert to floating point
		humidity_percent = (float) humData * 6.10e-3; // 100 / (2^14 - 1)
		temperature_c = (float) tempData * 1.007e-2 - 40.0; // 165 / (2^14 - 1)

		return status;
	}
};
#endif // USE_DHT_DEVICE


//============================================
// BMP085 BAROMETRIC PRESSURE AND TEMPERATURE SENSOR
//============================================


#ifdef USE_BMP085_DEVICE
class BMP085Device : public Device {
public:

	// initialize barometric pressure sensor; does not take pin since I2C; does take extra arguments
	BMP085Device() : Device( 0 ) {
		_bmp = NULL;
		_pressureAtSeaLevel = 101325;
	}

	BMP085Device( float pressureAtSeaLevel ) : Device( 0 ) {
		_bmp = NULL;
		_pressureAtSeaLevel = 101325;
		if(pressureAtSeaLevel > 0){
			_pressureAtSeaLevel = pressureAtSeaLevel;
		}
	}

	// initialize barometric pressure sensor; does not take pin since I2C;
	// arguments: pressure at sea level
	BMP085Device( char *args[], int argCount ) : Device( 0 ) {
		_bmp = NULL;
		_pressureAtSeaLevel = 101325; // Use 1 atm if no additional parameter is given
		if (argCount >= 1) {
			float args0 = (float) atof( args[ 0 ] );
			if(args0 > 0){
				_pressureAtSeaLevel = args0;
			}
		}
	}

	~BMP085Device(){
		delete _bmp;
	}

	// three values: temperature, pressure, and altitude
	int valueCount() { return 3; }

	// returns temperature (degrees C), pressure (pascals), and altitude (meters)
	float *values() {
		if (_bmp == NULL){
			init();
		}
		float temp = _bmp->readTemperature();
		long pressure = (long) _bmp->readPressure();
		float altitude = _bmp->readAltitude( _pressureAtSeaLevel );
		_values[0] = temp;
		_values[1] = (float) pressure;
		_values[2] = altitude;
		return _values;
	}

	// last temperature value (degrees C); assumes called refresh()
	inline float temperature() const { return _values[ 0 ]; }

	// last pressure value (pascals); assumes called refresh()
	inline float pressure() const { return _values[ 1 ]; }

	// last altitude value (meters); assumes called refresh()
	inline float altitude() const { return _values[ 2 ]; }

private:

	void init(){
		_bmp = new Adafruit_BMP085();
		_bmp->begin( BMP085_STANDARD );
	}

	// internal data
	Adafruit_BMP085 *_bmp;
	float _pressureAtSeaLevel;
	float _values[ 3 ];
};
#endif // USE_BMP085_DEVICE


//============================================
// BMP180 BAROMETRIC PRESSURE AND TEMPERATURE SENSOR
//============================================


#ifdef USE_BMP180_DEVICE
class BMP180Device : public Device {
public:

	// initialize barometric pressure sensor; does not take pin since I2C; does take extra arguments
	BMP180Device() : Device( 0 ) {
		_bmp = NULL;
		_pressureAtSeaLevel = 101325;
	}

	BMP180Device( float pressureAtSeaLevel ) : Device( 0 ) {
		_bmp = NULL;
		_pressureAtSeaLevel = 101325;
		if(pressureAtSeaLevel > 0){
			_pressureAtSeaLevel = pressureAtSeaLevel;
		}
	}

	// initialize barometric pressure sensor; does not take pin since I2C;
	// arguments: pressure at sea level
	BMP180Device( char *args[], int argCount ) : Device( 0 ) {
		_bmp = NULL;
		_pressureAtSeaLevel = 101325; // Use 1 atm if no additional parameter is given
		if (argCount >= 1) {
			float args0 = (float) atof( args[ 0 ] );
			if(args0 > 0){
				_pressureAtSeaLevel = args0;
			}
		}
	}

	~BMP180Device(){
		delete _bmp;
	}

	// three values: temperature, pressure, and altitude
	int valueCount() { return 3; }

	// returns temperature (degrees C), pressure (pascals), and altitude (meters)
	float *values() {
		if (_bmp == NULL){
			init();
		}
		float temp = _bmp->bmp085GetTemperature(_bmp->bmp085ReadUT());
		long pressure = (long) _bmp->bmp085GetPressure(_bmp->bmp085ReadUP());
		float altitude = calcAltitude( pressure, _pressureAtSeaLevel );
		_values[0] = temp;
		_values[1] = (float) pressure;
		_values[2] = altitude;
		return _values;
	}

	// last temperature value (degrees C); assumes called refresh()
	inline float temperature() const { return _values[ 0 ]; }

	// last pressure value (pascals); assumes called refresh()
	inline float pressure() const { return _values[ 1 ]; }

	// last altitude value (meters); assumes called refresh()
	inline float altitude() const { return _values[ 2 ]; }

private:

	void init(){
		_bmp = new Barometer();
		_bmp->init();
	}

	float calcAltitude(long pressure, float sealevelPressure) {
		// From Adafruit_BMP085.cpp
		float altitude;

		altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));

		return altitude;
	}

	// internal data
	Barometer *_bmp;
	float _pressureAtSeaLevel;
	float _values[ 3 ];
};
#endif // USE_BMP180_DEVICE


//============================================
// WIND SPEED
//============================================


#ifdef USE_WIND_DEVICES


// global variables used by interrupts
volatile unsigned long g_lastWindTime0 = 0;
volatile unsigned long g_lastWindTime1 = 0;
volatile float g_windSpeed0 = 0.0; // miles per hour
volatile float g_windSpeed1 = 0.0; // miles per hour


// interrupt for computing wind speed
void windSpeedInterrupt0() {
	unsigned long time = millis();
	int tDiff = time - g_lastWindTime0;
	if (tDiff > 20) { // if enough time since last time (debounce)
		g_windSpeed0 = 1492.0 / tDiff; // convert msec/pulse to mph
		g_lastWindTime0 = time;
	}
	if (tDiff > 1492){ // if pulses are too far apart, assume zero speed
		g_windSpeed0 = 0;
	}
}


// additional interrupt for computing wind speed
void windSpeedInterrupt1() {
	unsigned long time = millis();
	int tDiff = time - g_lastWindTime1;
	if (tDiff > 20) { // if enough time since last time (debounce)
		g_windSpeed1 = 1492.0 / tDiff; // convert msec/pulse to mph
		g_lastWindTime1 = time;
	}
	if (tDiff > 1492){ // if pulses are too far apart, assume zero speed
		g_windSpeed1 = 0;
	}
}


class WindSpeedDevice : public Device {
public:

	WindSpeedDevice( byte pin ) : Device( pin ) {
		_windSpeed = 0;
		if (pin == 2) {
			attachInterrupt( 0, windSpeedInterrupt0, RISING );
		} else if (pin == 3) {
			attachInterrupt( 1, windSpeedInterrupt1, RISING );
		} else {
			sendError(MEC_INVALID_PIN);
		}
	}

	// returns wind speed in meters per second
	float value() {
		// We need to check the times here because the interrupt won't trigger in the case
		// of an immediate stop. This is unlikely with actual wind, but we still want it to
		// say "0" if you grab it and stop it.
		if (_pin == 2) {
			if (millis() - g_lastWindTime0 > 1492)
				g_windSpeed0 = 0;
			_windSpeed = g_windSpeed0;
		} else if(_pin == 3) {
			if (millis() - g_lastWindTime1 > 1492)
				g_windSpeed1 = 0;
			_windSpeed = g_windSpeed1;
		}
		return _windSpeed * 0.447038; // m/s
	}

private:
	float _windSpeed;
};


//============================================
// WIND DIRECTION
//============================================


class WindDirectionDevice : public Device {
public:

	WindDirectionDevice( byte pin ) : Device( pin ) {
		_windDirs[0] = 786;
		_windDirs[1] = 406;
		_windDirs[2] = 460;
		_windDirs[3] = 84;
		_windDirs[4] = 92;
		_windDirs[5] = 66;
		_windDirs[6] = 184;
		_windDirs[7] = 127;
		_windDirs[8] = 287;
		_windDirs[9] = 244;
		_windDirs[10] = 631;
		_windDirs[11] = 600;
		_windDirs[12] = 946;
		_windDirs[13] = 827;
		_windDirs[14] = 890;
		_windDirs[15] = 702;
	}

	float value() {
		int windDirInput = analogRead( _pin );

		// compute wind direction by finding nearest value in table
		int bestDiff = 1024;
		int windDir = 0;
		for (int i = 0; i < 16; i++) {
			int diff = windDirInput - _windDirs[ i ];
			if (diff < 0)
				diff = -diff;
			if (diff < bestDiff) {
				bestDiff = diff;
				windDir = i;
			}
		}
		return windDir * 22.5;
	}

private:
	int _windDirs[ 16 ];
};
#endif // USE_WIND_DEVICES


//============================================
// RAIN GAUGE DEVICE
//============================================


#ifdef USE_RAIN_GAUGE_DEVICE


// global variables used by interrupts
volatile unsigned long g_lastRainTime0 = 0;
volatile unsigned long g_lastRainTime1 = 0;
volatile unsigned long g_rainCount0 = 0;
volatile unsigned long g_rainCount1 = 0;


// interrupt for computing rainfall
void rainInterrupt0() {
	unsigned long time = millis();
	int tDiff = time - g_lastRainTime0;
	if (tDiff > 100) { // if enough time since last time (debounce)
		g_rainCount0++;
		g_lastRainTime0 = time;
	}
}


// additional interrupt for computing rainfall
void rainInterrupt1() {
	unsigned long time = millis();
	int tDiff = time - g_lastRainTime1;
	if (tDiff > 100) { // if enough time since last time (debounce)
		g_rainCount1++;
		g_lastRainTime1 = time;
	}
}


class RainGaugeDevice : public Device {
public:

	RainGaugeDevice( byte pin ) : Device( pin ) {
		_totalRain = 0;
		_rainFactor = 0.011; // Each time the rain gauge tips, it's 0.011 in of rain
		if (pin == 2) {
			attachInterrupt( 0, rainInterrupt0, RISING );
		} else if (pin == 3) {
			attachInterrupt( 1, rainInterrupt1, RISING );
		} else {
			sendError(MEC_INVALID_PIN);
		}
	}

	float value() {
		int rainCount = 0;
		if (_pin == 2) {
			rainCount = g_rainCount0;
		} else if(_pin == 3) {
			rainCount = g_rainCount1;
		}
		_totalRain = rainCount * _rainFactor;
		return _totalRain;
	}

private:
	float _rainFactor;
	float _totalRain;
};
#endif


//============================================
// PING ULTRASONIC RANGER
//============================================


class PingDevice : public Device {
public:

	// optional timeout in milliseconds
	PingDevice( byte pin, int timeout = 10 ) : Device( pin ) {
		_timeout = timeout * 1000; // convert from milliseconds to microseconds
	}

	// optional argument: timeout in milliseconds
	PingDevice( byte pin, char *args[], int argCount ) : Device( pin ) {
		_timeout = 10000;
		if (argCount >= 1) {
			_timeout = atoi( args[ 0 ] ) * 1000; // convert from milliseconds to microseconds
		}
	}

	float value() {

		// start low for a clean pulse
		pinMode( _pin, OUTPUT );
		digitalWrite( _pin, LOW );
		delayMicroseconds( 2 );

		// pulse high
		digitalWrite( _pin, HIGH );
		delayMicroseconds( 5 );
		digitalWrite( _pin, LOW );

		// switch to input and wait for the pulse
		pinMode( _pin, INPUT );
		long duration = pulseIn( _pin, HIGH, _timeout );
		// 29 microseconds per centimeter. There and back so / 2
		return (float) duration / 29.0 / 2.0;
	}

private:
	unsigned long _timeout;
};


//============================================
// MAXBOTIX SERIAL RANGE
//============================================


#ifdef USE_MAXBOTIX_SERIAL_DEVICE
class MaxBotixSerialRangeDevice : public Device {
public:

	// initialize connection with sensor - true argument to _serial specifies inverted serial.
	MaxBotixSerialRangeDevice( byte pin ) : Device( pin ) , _serial( pin + 1, pin, true) {
		_distIndex = 0;
		// Initialize the _dist array
		for(int i = 0; i < 5; i++){
			_dist[i] = 0;
		}

		pinMode(_pin, OUTPUT);
		digitalWrite(pin, HIGH);
		pinMode(_pin+1, INPUT);
		_serial.begin(9600);
	}

	float value() {
		while(_serial.available()){
			char c = _serial.read();
			if(c == 'R'){
				_distIndex = 0;
			}else if(_distIndex < 4){
				_dist[_distIndex] = c;
				// If this is the last byte, then replace our current value with this new one
				if(_distIndex == 3){
					_value = (float)atoi(_dist);
				}
				_distIndex++;
			}
		}
		return _value;
	}

private:
	SoftwareSerial _serial;
	char _dist[5]; // 4 Distance characters and 1 terminating character
	int _distIndex;
	float _value;
};
#endif // USE_MAXBOTIX_SERIAL_DEVICE


//============================================
// ADXL345 ACCELEROMETER
//============================================


#ifdef USE_ADXL345_DEVICE
class ADXL345Device : public Device {
public:

	// initialize accelerometer; does not take pin since I2C
	ADXL345Device( byte scale = 0 ) : Device( 0 ) {
		_scale = scale;
		_adxl = NULL;
	}

	// initialize accelerometer; does not take pin since I2C;
	// optional argument: scale
	ADXL345Device( char *args[], int argCount ) : Device( 0 ) {
		_scale = 0;
		_adxl = NULL;
		if (argCount >= 1) {
			_scale = atoi( args[ 0 ] );
		}
	}

	~ADXL345Device() {
		delete _adxl;
	}

	// three values: x, y, and z acceleration
	int valueCount() { return 3; }

	// returns x, y, and z acceleration
	float *values() {
		if (_adxl == NULL){
			init();
		}
		byte scale = 0;
		_adxl->getRangeSetting(&scale);
		float gLSB = (scale * 2.0) / 1024.0;
		int x = 0, y = 0, z = 0;
		_adxl->readAccel( &x, &y, &z );
		_values[0] = x * gLSB;
		_values[1] = y * gLSB;
		_values[2] = z * gLSB;
		return _values;
	}

	// This function should be called with the device flat on a surface with no vibrations
	// or other movements. It will read the acceleration in all axes and calculate the
	// offset necessary to bring x and y as close to 0, and z as close to 1, as possible.
	void calibrate() {

		// get sum of 100 readings
		_adxl->setAxisOffset( 0, 0, 0 );
		float *deviceValues = NULL;
		float x = 0.0, y = 0.0, z = 0.0;
		for (int i = 0; i < 100; ++i) {
			deviceValues = values();
			x += deviceValues[0];
			y += deviceValues[1];
			z += deviceValues[2];
		}

		// 0.0156 g per bit
		x = (0 - (x / 100)) / 0.0156;
		y = (0 - (y / 100)) / 0.0156;
		z = (1 - (z / 100)) / 0.0156;
		_adxl->setAxisOffset( (int) x, (int) y, (int) z );
	}

	// simple accessors; assumes called refresh()
	inline float x() const { return _values[ 0 ]; }
	inline float y() const { return _values[ 1 ]; }
	inline float z() const { return _values[ 2 ]; }

private:

	// create and initialize internal device
	void init() {
		_adxl = new ADXL345();
		_adxl->powerOn();
		if (_scale > 0) {
			// The adxl library checks for valid range values, so we won't repeat that here
			_adxl->setRangeSetting( _scale );
		}
	}

	ADXL345 *_adxl;
	float _values[3];
	byte _scale;
};
#endif // USE_ADXL345_DEVICE


//============================================
// HMC5883L COMPASS
//============================================


#ifdef USE_HMC5883L_DEVICE
class HMC5883LDevice : public Device {
public:

	// scale must be one of: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1
	HMC5883LDevice( float scale ) : Device( 0 ) {
		_scale = scale;
		_mMeter = NULL;
	}

	// initialize compass; does not take pin since I2C;
	// option argument: scale (must be one of: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1)
	HMC5883LDevice( char *args[], int argCount ) : Device( 0 ) {
		float scale = 2.5;
		if (argCount >= 1) {
			scale = atof( args[ 0 ] );
		}
		_scale = scale;
		_mMeter = NULL;
	}
	~HMC5883LDevice() {
		delete _mMeter;
	}

	// three values: x, y, and z magnetic field components
	int valueCount() { return 3; }

	// returns x, y, and z magnetic field components
	float *values() {
		if (_mMeter == NULL){
			init();
		}
		MagnetometerScaled _scaled = _mMeter->ReadScaledAxis();
		_values[0] = _scaled.XAxis;
		_values[1] = _scaled.YAxis;
		_values[2] = _scaled.ZAxis;
		// Just for debugging
		// MagnetometerRaw _raw = _mMeter->ReadRawAxis();
		// _values[0] = _raw.XAxis;
		// _values[1] = _raw.YAxis;
		// _values[2] = _raw.ZAxis;
		return _values;
	}

	// simple accessors; assumes called refresh()
	inline float x() const { return _values[ 0 ]; }
	inline float y() const { return _values[ 1 ]; }
	inline float z() const { return _values[ 2 ]; }

private:

	void init() {
		Wire.begin();
		_mMeter = new HMC5883L();
		int success = _mMeter->SetScale(_scale);
		if(success != 0){
			_scale = 2.5; // This is our default
			_mMeter->SetScale(_scale);
		}
		_mMeter->SetMeasurementMode(Measurement_Continuous);
		// Just for debugging
		// Serial.print(" Res: ");
		// Serial.println(_mMeter->getResolution());
	}


	HMC5883L *_mMeter;
	float _values[3];
	float _scale;
};
#endif // USE_HMC5883L_DEVICE


//============================================
// INA219 CURRENT SENSOR DEVICE
//============================================


#ifdef USE_INA219_DEVICE
class INA219Device : public Device {
public:

	// max amps must be 1 or 2
	INA219Device( int maxAmps = 2 ) : Device( 0 ) {
		_ina219 = NULL;
		_maxAmps = maxAmps;
	}

	// initialize current sensor; does not take pin since I2C;
	// optional argument: maxAmps (1 or 2)
	INA219Device( char *args[], int argCount ) : Device( 0 ) {
		_ina219 = NULL;
		if (argCount >= 1) {
			_maxAmps = atoi( args[ 0 ] );
		}
	}

	~INA219Device() {
		delete _ina219;
	}

	// three values: bus voltage (V), shunt voltage (mV), and current (mA)
	int valueCount() { return 3; }

	// returns current (A), bus voltage (V), and shunt voltage (V)
	float *values() {
		if (_ina219 == NULL){
			init();
		}
		_values[0] = _ina219->getCurrent_mA() * 0.001; // The current, derived via Ohms Law from the measured shunt voltage.
		_values[1] = _ina219->getBusVoltage_V(); // The total voltage seen by the circuit under test.  (Supply voltage - shunt voltage).
		_values[2] = _ina219->getShuntVoltage_mV() * 0.001; // The voltage between V- and V+.  This is the measured voltage drop across the shunt resistor.

		// Some of our sensors go out of range when idle. We want to check for this and assume everything is 0 when this is the case.
		if (_values[2] > _maxAmps) {
			_values[0] = 0.0;
			_values[1] = 0.0;
			_values[2] = 0.0;
		}
		return _values;
	}

	// last current (A); assumes called refresh()
	inline float current() const { return _values[ 0 ]; }

	// last bus voltage (V); assumes called refresh()
	inline float busVoltage() const { return _values[ 1 ]; }

	// last shunt voltage (V); assumes called refresh()
	inline float shuntVoltage() const { return _values[ 2 ]; }

private:

	void init() {

		_ina219 = new Adafruit_INA219();
		_ina219->begin();
		if (_maxAmps < 1)
			_maxAmps = 1;
		if (_maxAmps > 2)
			_maxAmps = 2;
		if (_maxAmps == 1) {
			// This was a private function in the library with no way to cause it to be called.
			// We've made it public so we can configure the calibration.
			_ina219->ina219SetCalibration_32V_1A();
		}
	}

	float _values[3];
	int _maxAmps;
	Adafruit_INA219 *_ina219;
};
#endif // USE_INA219_DEVICE


//============================================
// GROVE IR TEMP DEVICE
//============================================


#ifdef USE_GROVE_IR_TEMP_DEVICE
class IRTempDevice : public Device {
public:

	// pin is an analog pin;
	// optional argument: voltage offset
	IRTempDevice( byte pin, float voltageOffset = 0 ) : Device( pin ) {
		_voltageOffset = voltageOffset;
	}

	// pin is an analog pin;
	// optional argument: voltage offset
	IRTempDevice( byte pin, char *args[], int argCount ) : Device( pin ) {
		_voltageOffset = 0.0;
		if (argCount >= 1) {
			_voltageOffset = atof( args[ 0 ] );
		}
	}

	// values are [Object Temp, Ambient Temp, Sensor Voltage (Used for calibration)]
	int valueCount() { return 3; }

	// number of decimal places to use when sending data
	int decimalPlaces() { return 3; }

	// returns object temp (degrees C), ambient temp (degrees C), and sensor voltage
	float *values() {

		// To get an accurate reading we need to set the reference voltage to 1.1
		#ifdef __AVR_ATmega2560__
			analogReference( INTERNAL1V1 );
		#else
			analogReference( INTERNAL );
		#endif

		// Throw away a few readings (50 = 5ms) while the ADC settles to the new reference voltage
		for (int i = 0; i < 50; ++i)
		{
			analogRead(_pin);
		}
		// Take our readings
		// We need to use the sensor temp to get the object temp, so we'll calculate it first
		_values[1] = measureSensorTemp();
		_values[0] = measureObjectTemp(_values[1]);
		// Check bounds here. We only measure between -10 and 100 C
		// if (_values[0] > 100){
		//   _values[0] = 100;
		// }
		// if (_values[0] < -10){
		//   _values[0] = -10;
		// }
		// Then change it back for other sensors
		analogReference(DEFAULT);
		// Then throw away multiple readings again so any sensors that follow don't have to deal with this.
		for (int i = 0; i < 50; ++i)
		{
			analogRead(_pin);
		}
		return _values;
	}

	float measureSensorTemp(){
		float B = 3694.0f; // B parameter from the datasheet
		float T0 = 298.15f; // Temp in K at R0
		float R0 = 100000.0f; // From the datasheet
		float analogValue = 0;
		for (int i = 0; i < 10; ++i)
		{
			analogValue += float(analogRead(_pin));
			delay(1);
		}
		analogValue /= 10;
		// It seems that:
			// 1.1 is the ADC reference voltage
			// 2.5 is input voltage
			// 2,000,000 is the balancing resistor

		// These calculations are in the Seeed sample code, but I'm not sure what they're derived from so...
		//float resistance = analogValue * 1.1 / 1023;
		//resistance = 2000000 * resistance / (2.5 - resistance);

		// This calculation is using the voltage divider equation for R2 and gives similar values to Seeed's just above.
		float resistance = 2000000 / (( 2.5 / ( analogValue * 1.1/1023 )) - 1 );

		// Calculate Temp - From the Arduino Playground page on thermistors
		float temperature = 1.0f / ( 1.0f / T0 + ( 1.0f / B ) * log( resistance / R0 ));
		// Convert K to C and return
		return temperature - 273.15;

	}

	float measureObjectTemp(float sensorTemp){
		float analogValue = 0;
		for (int i = 0; i < 10; ++i){
			analogValue += float(analogRead(_pin+1)); // The IR sensor is on pin+1
			delay(1);
		}
		analogValue /= 10;

		// Taken from the Seeed sample code. Not sure where they're getting this
		// 0.5 from. They call it 'reference_vol'.
		float sensorVoltage = (analogValue * 1.1 / 1023) - (0.5 + _voltageOffset);
		_values[2] = sensorVoltage;

		float amplifier_coefficient = 100.0;
		int sensorTempIndex = ( sensorTemp / 10 ) + 1;
		float sensor_mV = (sensorVoltage * 1000.0) / amplifier_coefficient;
		int objectTempIndex = 0;
		for(objectTempIndex = 0; objectTempIndex < 13; ++objectTempIndex){
			if(( sensor_mV > (_objectTempTable[objectTempIndex][sensorTempIndex] / 1000.0f) ) && ( sensor_mV < (_objectTempTable[objectTempIndex+1][sensorTempIndex] / 1000.0f) ))
			{
				// We've found the objectTempIndex
				break;
			}
		}
		// Again, taken from sample code. Quite convoluted.
		float objectTempTableValue = _objectTempTable[objectTempIndex][(int)( sensorTemp / 10 ) + 1] / 1000.0f;
		float objectTempTableValuePlusOne = _objectTempTable[objectTempIndex + 1][(int)( sensorTemp / 10 ) + 1] / 1000.0f;
		float objectTemp = ( 10 * sensor_mV ) / (objectTempTableValuePlusOne - objectTempTableValue);
		float finalObjectTemp = sensorTemp + objectTemp;
		return finalObjectTemp;
	}

	// last object temperature (degrees C); assumes called refresh()
	inline float objectTemperature() const { return _values[ 0 ]; }

	// last ambient temperature (degrees C); assumes called refresh()
	inline float ambientTemperature() const { return _values[ 1 ]; }

	// last sensor voltage (for calibration); assumes called refresh()
	inline float sensorVoltage() const { return _values[ 2 ]; }

private:
	float _values[3];
	float _voltageOffset;
	// This is a table of sensor temp vs object temp.
	// Each element is a voltage in microVolts
	static int _objectTempTable[13][12];
	uint8_t _referenceVoltage;
};

int IRTempDevice::_objectTempTable[13][12] = {
/*0*/	{ 0,-274,-580,-922,-1301,-1721,-2183,-2691,-3247,-3854,-4516,-5236},
/*1*/	{ 271,0,-303,-642,-1018,-1434,-1894,-2398,-2951,-3556,-4215,-4931},
/*2*/	{ 567,300,0,-335,-708,-1121,-1577,-2078,-2628,-3229,-3884,-4597},
/*3*/	{ 891,628,331,0,-369,-778,-123,-1728,-2274,-2871,-3523,-4232},
/*4*/	{ 1244,985,692,365,0,-405,-853,-1347,-1889,-2482,-313,-3835},
/*5*/	{ 1628,1372,1084,761,401,0,-444,-933,-147,-2059,-2702,-3403},
/*6*/	{ 2043,1792,1509,1191,835,439,0,-484,-1017,-1601,-224,-2936},
/*7*/	{ 2491,2246,1968,1655,1304,913,479,0,-528,-1107,-174,-2431},
/*8*/	{ 2975,2735,2462,2155,1809,1424,996,522,0,-573,-1201,-1887},
/*9*/	{ 3495,3261,2994,2692,2353,1974,1552,1084,568,0,-622,-1301},
/*10*/	{ 4053,3825,3565,327,2937,2564,2148,1687,1177,616,0,-673},
/*11*/	{ 4651,443,4177,3888,3562,3196,2787,2332,1829,1275,666,0},
/*12*/	{ 529,5076,483,4549,4231,3872,347,3023,2527,198,1379,720}
};

#endif // USE_GROVE_IR_TEMP_DEVICE


//============================================
// IR ENCODER
//============================================


#ifdef USE_IR_ENCODER_DEVICE


// encoder interrupts
volatile unsigned long _pos0 = 0;
volatile unsigned long _pos1 = 0;
void irEncoderChange0() {
	_pos0++;
}
void irEncoderChange1() {
	_pos1++;
}


class IREncoderDevice : public Device {
public:

	// initialize connection with sensor; pin must be 2 or 3
	IREncoderDevice( byte pin, float encoderCirc = 20.0, int numSegments = 40 ) : Device( pin ) {
		_encoderCirc = encoderCirc;
		_numSegments = numSegments;
		init();
	}

	// initialize connection with sensor; pin must be 2 or 3
	IREncoderDevice( byte pin, char *args[], int argCount ) : Device( pin ) {
		_encoderCirc = 20; // cm - We're assuming 20 for now.
		_numSegments = 40; // # Transitions
		if (argCount >= 1) {
			_encoderCirc = atof( args[ 0 ] );
		}
		if (argCount >= 2) {
			_numSegments = atoi( args[ 1 ] );
		}
		init();
	}

	void init() {
		if (_pin == 2)
			attachInterrupt( 0, irEncoderChange0, CHANGE );
		else if (_pin == 3)
			attachInterrupt( 1, irEncoderChange1, CHANGE );
		else
			sendError( MEC_INVALID_PIN );
		_distPerSegment = _encoderCirc / _numSegments;
		_lastPos = 0;
	}

	int decimalPlaces() { return 4; };
	int valueCount() { return 3; }

	// returns three values: (1) position in encoder ticks, (2) position in cm, (3) velocity in cm/ms
	float *values() {
		unsigned long now = millis();
		if(_firstCheck){
			_lastCheck = now;
			_firstCheck = false;
		}
		int position = _pos0;
		if(_pin == 3){
			position = _pos1;
		}
		_values[0] = position; // Transitions
		_values[1] = position * _distPerSegment / 100; // Total linear distance in meters
		int diff = position - _lastPos;
		if(!_firstCheck){
			_values[2] = _distPerSegment * diff / (now - _lastCheck) * 10; // m/s
		}else{
			_values[2] = 0;
		}
		// If the positions haven't changed, then we're not moving. Send a vel
		// of 0 and reset the last time out change interrupt was triggered.
		_lastPos = position;
		_lastCheck = now;
		return _values;
	}

	// last encoder position (in encoder ticks); assumes called refresh()
	inline float position() const { return _values[ 0 ]; }

	// last computed distance in m; assumes called refresh()
	inline float distance() const { return _values[ 1 ]; }

	// last computed velocity in m/s; assumes called refresh()
	inline float velocity() const { return _values[ 2 ]; }

private:

	// internal data
	float _values[3];
	int _numSegments;
	float _distPerSegment;
	float _encoderCirc;
	int _lastPos;
	bool _firstCheck;
	unsigned long _lastCheck;
};
#endif // USE_IR_ENCODER_DEVICE


//============================================
// RF TRANSMITTER AND RECEIVER
//============================================


#ifdef USE_RF_DEVICE
class RFReceiverDevice : public Device {
public:

	// optional argument: timeout (in milliseconds)
	RFReceiverDevice( byte pin, uint16_t bitsPerSecond = 2000 ) : Device( pin ) {
		init(pin, bitsPerSecond);
	}

	// optional argument: timeout (in milliseconds)
	RFReceiverDevice( byte pin, char *args[], int argCount ) : Device( pin ) {
		uint16_t bitsPerSecond = 2000;
		if (argCount >= 1) {
			bitsPerSecond = atoi(args[0]);
		}
		init(pin, bitsPerSecond);
	}

	void init(byte pin, uint16_t bitsPerSecond) {
		_bitsPerSecond = bitsPerSecond;
		vw_set_rx_pin(pin);
			vw_setup(_bitsPerSecond);
			vw_rx_start();   // Start the receiver
	}

	float value() {
		uint8_t len = 4; // bytes in float
		byte buffer[4];
		if( vw_have_message() && vw_get_message(buffer, &len) ){
			// Make sure we read 4 bytes
			if(len == 4){
				_valueUnion.bytes[0] = buffer[0];
				_valueUnion.bytes[1] = buffer[1];
				_valueUnion.bytes[2] = buffer[2];
				_valueUnion.bytes[3] = buffer[3];
			}
		}
		return _valueUnion.value;
	}

	int valueCount() { return 1; }

private:
	uint16_t _bitsPerSecond;

	union{
		byte bytes[4]; // 4 bytes in float
		float value;
	} _valueUnion;
};


class RFTransmitterDevice : public Device {
public:

	// optional argument: timeout (in milliseconds)
	RFTransmitterDevice( byte pin, uint16_t bitsPerSecond = 2000 ) : Device( pin ) {
		init(pin, bitsPerSecond);
	}

	// optional argument: timeout (in milliseconds)
	RFTransmitterDevice( byte pin, char *args[], int argCount ) : Device( pin ) {
		uint16_t bitsPerSecond = 2000;
		if (argCount >= 1) {
			bitsPerSecond = atoi(args[0]);
		}
		init(pin, bitsPerSecond);
	}

	void init(byte pin, uint16_t bitsPerSecond) {
		_valueUnion.value = 0.0f;
		_bitsPerSecond = bitsPerSecond;
		vw_set_tx_pin(pin);
			vw_setup(_bitsPerSecond);
	}

	float value() {
		// Send in value because this is called continually.
		// Check if the transmitter is ready
		if(!vx_tx_active()){
			uint8_t len = 4; // 4 bytes in float
			vw_send(_valueUnion.bytes, len);
		}
		return _valueUnion.value;
	}

	void setValue(float value) {
		// Just store the value here. It will be sent in value()
		_valueUnion.value = value;
	}

	int valueCount() { return 1; }

	void transmit() { value(); }

private:
	uint16_t _bitsPerSecond;

	union{
		byte bytes[4]; // 4 bytes in float
		float value;
	} _valueUnion;
};
#endif


//============================================
// MINI PH SENSOR
//============================================


#ifdef USE_MINI_PH_DEVICE
class MiniPhDevice : public Device {
public:

	// initialize connection with sensor; does not take pin since I2C
	MiniPhDevice( char address = 0x4D ) : Device( 0 ) {
		_initDone = false;
		_address = address;
		setDefaults();
	}

	// initialize connection with sensor; does not take pin since I2C
	MiniPhDevice( char *args[], int argCount ) : Device( 0 ) {
		if (argCount >= 1) {
			char address = (char) strtol(args[ 0 ], NULL, 16);
			_address = address;
		}
		_initDone = false;
		setDefaults();
	}

	float value() {

		if (_initDone == false)
			init();

		float value = -1;
		Wire.requestFrom(_address, 2);
		if (Wire.available() >= 2) {
			int adcHigh = Wire.read();
			int adcLow = Wire.read();
			value = (adcHigh * 256) + adcLow;
		}
		return value;
	}

	float pH() {
		float mV = value() / _ADCSteps * _vRef * _mVConversion;
		float temp = ((_vRef * _pH7 / _ADCSteps * _mVConversion) - mV) / _opAmpGain;
		return 7 - temp / _pHSlope;
	}

	void setCalib7( float rawValue ){
		_pH7 = rawValue;
		calibrate();
	}
	void setCalib4( float rawValue ){
		_pH4 = rawValue;
		calibrate();
	}

	char getAddress(){
		return _address;
	}

private:

	char _address;
	boolean _initDone;
	float _pH7;
	float _pH4;
	float _pHSlope;
	float _vRef;
	float _opAmpGain;
	int _ADCSteps;
	int _mVConversion;

	void init(){
		Wire.begin();
		_initDone = true;
	}

	void calibrate(){
		float pHDiff = _pH7 - _pH4;
		int pHSteps = 3; // 7 - 4 = 3 steps between calibration points
		_pHSlope = _vRef * pHDiff / _ADCSteps * _mVConversion / _opAmpGain / pHSteps;
	}

	void setDefaults(){
		// Default values
		_pH7 = 2048;
		_pH4 = 1286;
		_pHSlope = 59.16;

		_opAmpGain = 5.25;
		_ADCSteps = 4096; // Number of steps in 12 Bit ADC
		_mVConversion = 1000; // mV in V
		// This is the vRef into the ADC on the MinipH board. It's dependant on
		// VCC and not exact. Should be mesaured and adjusted.
		_vRef = 4.096;
	}
};
#endif // USE_MINI_PH_DEVICE


//============================================
// GPS DEVICE
//============================================


#ifdef USE_GPS_DEVICE


#define GPS_BUF_LEN 60


// convert a 2-digit string into an integer
int atoi2( char *str ) {
	int d1 = str[ 0 ] - '0';
	int d2 = str[ 1 ] - '0';
	return d1 * 10 + d2;
}


// convert latitude/longitude string to float
float convertGps( char *str ) {
	float val = 0;
	int dotPos = indexOf( str, '.' );
	if (dotPos > 2) {
		float minutes = atof( str + dotPos - 2 );
		str[ dotPos - 2 ] = 0;
		int degrees = atoi( str );
		val = degrees + minutes / 60.0;
	}
	return val;
}


// split a string into parts
// destructive; rewrites orginal string (replacing delimiters with zeros)
// assumes maxPartCount >= 2
// fix(clean): should we keep this here? we could remove it and use a simpler comma-searching function
int splitString( char *str, char **parts, int maxPartCount ){
	parts[ 0 ] = str;
	int strIndex = 0;
	int partIndex = 1;
	while (str[ strIndex ]) {
		if (str[ strIndex ] == ',') {
			parts[ partIndex ] = str + strIndex + 1;
			str[ strIndex ] = 0;
			partIndex++;
			if (partIndex >= maxPartCount) {
				break;
			}
		}
		strIndex++;
	}
}


// the following code is adapted from the Arduino Time library
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24UL)
#define LEAP_YEAR(Y)  ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )
static const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};
unsigned long makeTime( int _year, int _month, int _day, int _hour, int _minute, int _second ){

	// the follow code assumes year is w.r.t. 1970
	_year -= 1970;

	// seconds from 1970 till 1 jan 00:00:00 of the given year
	unsigned long seconds = _year *(SECS_PER_DAY * 365UL);
	for (int i = 0; i < _year; i++) {
		if (LEAP_YEAR(i)) {
			seconds +=  SECS_PER_DAY; // add extra days for leap years
		}
	}

	// add days for this year, months start from 1
	for (int i = 1; i < _month; i++) {
		if ((i == 2) && LEAP_YEAR(_year)) {
			seconds += SECS_PER_DAY * 29UL;
		} else {
			seconds += SECS_PER_DAY * monthDays[ i - 1 ]; // monthDay array starts from 0
		}
	}
	seconds += (_day - 1) * SECS_PER_DAY;
	seconds += _hour * SECS_PER_HOUR;
	seconds += _minute * SECS_PER_MIN;
	seconds += _second;
	return seconds;
}


class GPSDevice : public Device {
public:

	// initialize the GPS device; pin specifies first pin of serial port (using standard software serial RX/TX convention)
	GPSDevice( byte pin ) : Device( pin ), _serial( pin, pin + 1 ) {
		#ifdef __AVR_ATmega2560__
		if (pin == 18) {
			Serial1.begin( 9600 );
		} else {
			_serial.begin( 9600 );
		}
		#else
		_serial.begin( 9600 );
		#endif
		_firstGpsTimestamp = 0;
		_firstGpsTime = 0;
		_showGpsMessages = false;
		_bufPos = 0;
		_values[ 0 ] = 0;
		_values[ 1 ] = 0;
		_values[ 2 ] = 0;
	}

	// check for new messages from the GPS device
	void check() {
		Stream &s = serial();
		while (s.available()) {
			char c = s.read();
			if (c >= 32) {
				if (_bufPos < GPS_BUF_LEN)
				_buf[ _bufPos++ ] = c;
			} else if (_bufPos) {
				_buf[ _bufPos ] = 0;
				processGpsMessage();
				_bufPos = 0;
			}
		}
	}

	// latitude, longitude, and number of satellites (most recent values)
	float *values() { check(); return _values; }

	// most recent latitude
	float latitude() { return _values[ 0 ]; }

	// most recent longitude
	float longitude() { return _values[ 1 ]; }

	// most recent satellite count
	float satellites() { return _values[ 2 ]; }

	// unix timestamp computed from first GPS timestamp and internal clock (may drift)
	unsigned long timestamp() { return _firstGpsTimestamp + (millis() - _firstGpsTime) / 1000; }

	// set whether or not to display diagnostic messages
	void showGpsMessages( bool show ) { _showGpsMessages = show; }

private:

	// get serial port
	Stream &serial() {
		#ifdef __AVR_ATmega2560__
		if (_pin == 18) {
			return Serial1;
		}
		#endif
		return _serial;
	}

	// process a serial message received from GPS device
	void processGpsMessage() {
		_buf[ 6 ] = 0;
		if (strEq( _buf, "$GPGSV" )) {
			if (_showGpsMessages) {
				Serial.print( F("GPS-GSV: ") );
				Serial.println( _buf + 7 );
			}
			char *parts[ 3 ];
			int partCount = splitString( _buf + 7, parts, 3 );
			if (partCount >= 3) {
				int satelliteCount = atoi( parts[ 2 ] ); // satellite count
				_values[ 2 ] = (float) satelliteCount;
				if (_showGpsMessages) {
					Serial.print( F("satellite count: ") );
					Serial.println( satelliteCount );
				}
			}
		} else if (strEq( _buf, "$GPRMC" )) {
			if (_showGpsMessages) {
				Serial.print( F("GPS-RMC: ") );
				Serial.println( _buf + 7 );
			}
			char *parts[ 9 ]; // we're expecting up to 12 parts, but we only use the first 9
			int partCount = splitString( _buf + 7, parts, 9 );
			if (partCount >= 9) {
				char *time = parts[ 0 ];
				char *date = parts[ 8 ];
				char *valid = parts[ 1 ];
				if (valid[ 0 ] == 'A') {
					float lat = convertGps( parts[ 2 ] );
					float lng = convertGps( parts[ 4 ] );
					if (parts[ 3 ][ 0 ] == 'S') // use negative for southtern hemisphere
						lat = -lat;
					if (parts[ 5 ][ 0 ] == 'W') // use negative for western hemisphere
						lng = -lng;
					if (lat && lng) { // note: we're assuming that we're not at exactly lat == 0 or lng == 0
						_values[ 0 ] = lat;
						_values[ 1 ] = lng;
					}
				}
				if (time[ 0 ] && date[ 0 ]) {
					int tHour = atoi2( time );
					int tMinute = atoi2( time + 2 );
					int tSecond = atoi2( time + 4 );
					int tDay = atoi2( date );
					int tMonth = atoi2( date + 2 );
					int tYear = 2000 + atoi2( date + 4 );
					if (_firstGpsTimestamp == 0) {
						_firstGpsTimestamp = makeTime( tYear, tMonth, tDay, tHour, tMinute, tSecond );
						_firstGpsTime = millis();
					}
				}
			}
		}
	}

	// unix timestamp of first timestamp from GPS
	unsigned long _firstGpsTimestamp;

	// milliseconds since Arduino startup corresponding to first GPS time
	unsigned long _firstGpsTime;

	// if true, show diagnostic messages
	bool _showGpsMessages;

	// a buffer of data
	char _buf[ GPS_BUF_LEN ];
	int _bufPos;

	// latitude, longitude, and number of satellites
	float _values[ 3 ];

	// serial port for communicating with GPS device
	SoftwareSerial _serial;
};
#endif // USE_GPS_DEVICE


//============================================
// ANALOG AVERAGE
//============================================


// take average of analog values (semi-irregularly sampled between updates)
class AnalogAverageDevice : public Device {
public:

	// initialize connection with sensor
	AnalogAverageDevice( byte pin ) : Device( pin ) {
		m_signalSum = 0;
		m_signalCount = 0;
	}

	// accumulate analog readings
	void check() {
		m_signalSum += analogRead( _pin );
		m_signalCount++;
	}

	// compute average of accumulated readings
	float value() {
		check(); // make sure we have at least one sample
		int mean = m_signalSum / m_signalCount; // compute integer mean
		m_signalSum = 0;
		m_signalCount = 0;
		return (float) mean;
	}

private:

	// accumulated signal values
	long m_signalSum;
	int m_signalCount;
};


//============================================
// PULSE COUNTER
//============================================


// pulse counts cooresponding to INT0 and INT1
volatile int m_pulseCount0 = 0;
volatile int m_pulseCount1 = 0;


// increment pulse count (interrupt attached to INT0)
void incrementPulseCount0() {
	m_pulseCount0++;
}


// increment pulse count (interrupt attached to INT1)
void incrementPulseCount1() {
	m_pulseCount1++;
}


// count pulses on a digital input pin (e.g. for geiger counter)
class PulseCounterDevice : public Device {
public:

	// initialize connection with sensor
	PulseCounterDevice( byte pin ) : Device( pin ) {
		if (pin == 2)
			attachInterrupt( 0, incrementPulseCount0, RISING );
		else if (pin == 3)
			attachInterrupt( 1, incrementPulseCount1, RISING );
		else
			sendError( MEC_INVALID_PIN );
	}

	// get pulse count since last update
	float value() {
		int count = 0;
		if (_pin == 2) {
			count = m_pulseCount0;
			m_pulseCount0 = 0;
		} else {
			count = m_pulseCount1;
			m_pulseCount1 = 0;
		}
		return (float) count;
	}
};


//============================================
// BUTTON SET
//============================================


// a set of buttons represented as a single numeric value (0 = no button, 1 = button 1, 2 = button 2, etc.)
class ButtonSetDevice : public Device {
public:

	// initialize connection with sensor;
	// args: list of pins
	ButtonSetDevice( byte pin, char *args[], int argCount ) : Device( pin ) {
		_buttonPressed = 0;
		_lastButtonPressed = 0;
		_buttonCount = 1;
		_buttonPins[ 0 ] = pin;
		if (argCount > 9)
			argCount = 9;
		pinMode( pin, INPUT );
		for (int i = 0; i < argCount; i++) {
			byte bPin = atoi( args[ i ] );
			_buttonPins[ _buttonCount++ ] = bPin;
			pinMode( bPin, INPUT );
		}
	}

	// create a button set using a collection of button input pins (array of integers)
	ButtonSetDevice( int *buttonPins, int buttonCount ) : Device( buttonPins[ 0 ] ) {
		_buttonPressed = 0;
		_lastButtonPressed = 0;
		if (buttonCount > 9)
			buttonCount = 9;
		_buttonCount = buttonCount;
		for (int i = 0; i < buttonCount; i++) {
			byte pin = buttonPins[ i ];
			_buttonPins[ i ] = pin;
			pinMode( pin, INPUT );
		}
	}

	// determine which button is pressed (if any)
	void check() {
		for (int i = 0; i < _buttonCount; i++) {
			if (digitalRead( _buttonPins[ i ] )) {
				_buttonPressed = i + 1;
				break;
			}
		}
	}

	// get latest button press (does not repeat a button value unless it was released)
	float value() {
		int val = 0;
		check();
		if (_buttonPressed && _buttonPressed != _lastButtonPressed) { // avoid repeat; could still repeat if bounce across update
			val = _buttonPressed;
		}
		_lastButtonPressed = _buttonPressed;
		_buttonPressed = 0;
		return val;
	}

private:

	// internal data
	unsigned char _buttonPins[ 10 ];
	unsigned char _buttonCount;
	unsigned char _buttonPressed;
	unsigned char _lastButtonPressed;
};


//============================================
// CHAINABLE RGB LED
//============================================


// RGB LED
// this class uses code from the seeed wiki: http://www.seeedstudio.com/wiki/index.php?title=Twig_-_Chainable_RGB_LED
class ChainableRgbLedDevice : public Device {
public:

	// initialize connection with sensor
	ChainableRgbLedDevice( byte pin ) : Device( pin ) {
		pinMode( _pin, OUTPUT );
		pinMode( _pin + 1, OUTPUT );
	}

	// send color value to LED:
	// values[ 0 ] = red
	// values[ 1 ] = green
	// values[ 2 ] = blue
	void setValues( float *values ) {
		_values[0] = values[0];
		_values[1] = values[1];
		_values[2] = values[2];
		uint8 r = (int) (_values[ 0 ] + 0.5f); // round to nearest (assuming positive)
		uint8 g = (int) (_values[ 1 ] + 0.5f);
		uint8 b = (int) (_values[ 2 ] + 0.5f);
		sendData( 0 ); // begin
		sendColor( r, g, b ); // note: can send multiple colors here if chained
		sendData( 0 ); // end
	}

	int valueCount() { return 3; };

	// returns red, green, blue
	float *values() {
		return _values;
	}

private:

	float _values[3];

	// one clock pulse: momentarily lower the clock line
	void clockStep() {
		digitalWrite( _pin, LOW );
		delayMicroseconds( 20 );
		digitalWrite( _pin, HIGH );
		delayMicroseconds( 20 );
	}

	// send 32-bits of data
	void sendData( uint32 dx ) {
		for (uint8 i=0; i<32; i++) {
			if ((dx & 0x80000000) != 0) {
				digitalWrite( _pin + 1, HIGH);
			} else {
				digitalWrite( _pin + 1, LOW);
			}
			dx <<= 1;
			clockStep();
		}
	}

	// compute negated code from top bits
	uint8 antiCode( uint8 data ) {
		uint8 anti = 0;
		if ((data & 0x80) == 0)
			anti |= 0x02;
		if ((data & 0x40) == 0)
			anti |= 0x01;
		return data;
	}

	// send a color to LED
	void sendColor( uint8 r, uint8 g, uint8 b ) {
		uint32 d = 0;
		d |= (uint32) 0x03 << 30;
		d |= (uint32) antiCode( b ) << 28;
		d |= (uint32) antiCode( g ) << 26;
		d |= (uint32) antiCode( r ) << 24;
		d |= (uint32) b << 16;
		d |= (uint32) g << 8;
		d |= r;
		sendData( d );
	}
};


//============================================
// GROVE DUST
//============================================


// Grove dust sensor
// fix(soon): rewrite this to use interrupts?
class GroveDustDevice : public Device {
public:

	// initialize connection with sensor
	GroveDustDevice( byte pin ) : Device( pin ) {
		_sampleTime = 30000;
		_lastConcentration = 0;
		resetValues();
	}

	void check() {
		if (_readStartTime == 0){
			_readStartTime = millis();
		}

		// check to see if we've hit the 30sec mark
		if ((millis() - _readStartTime) > _sampleTime){
			_lastConcentration = computeDustLevel();
			resetValues();
		}

		// check to see if the pin is LOW
		if (digitalRead(_pin) == LOW){
			if (_lastReadLow){
				// Our last read was LOW too, so add to the duration
				_lowDuration += millis() - _lastLowTime;
			} else {
				// Our last read was HIGH so start a new LOW cycle here
				_lastReadLow = true;
				_lastLowTime = millis();
			}
		} else {
			_lastReadLow = false;
		}
	}

	float value() {
		return _lastConcentration;
	}

private:

	void resetValues() {
		_lastReadLow = false;
		_lastLowTime = 0;
		_readStartTime = 0;
		_lowDuration = 0;
	}

	float computeDustLevel() {
		// Get the ratio of LOW to HIGH in the sample
		float ratio = _lowDuration / (_sampleTime * 10.0); // Percentage [1,100]
		return 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // From sensor spec sheet - via www.howmuchsnow.com/arduino/airquality/grovedust/
	}

	// internal data
	bool _lastReadLow;
	unsigned long _lastLowTime;
	unsigned long _readStartTime;
	unsigned long _lowDuration;
	unsigned long _sampleTime;
	float _lastConcentration;
};


//============================================
// HEART RATE
//============================================


volatile unsigned long _lastPulse0 = 0;
volatile unsigned long _lastPulse1 = 0;
volatile float _currentHeartRate0 = 0.0;
volatile float _currentHeartRate1 = 0.0;


// Calculate the instantaneous heart rate
void instHeartRate0() {
		unsigned long pulseWidth = millis() - _lastPulse0;
		// Milliseconds in a second / time between pulses
		if (pulseWidth != 0) {
			_currentHeartRate0 = 60000 / pulseWidth;
		}
		_lastPulse0 = millis();
}
void instHeartRate1(){
		unsigned long pulseWidth = millis() - _lastPulse1;
		// Milliseconds in a second / time between pulses
		if (pulseWidth != 0) {
			_currentHeartRate1 = 60000 / pulseWidth;
		}
		_lastPulse1 = millis();
}


// This device is designed to work with the Grove chest strap heart rate sensor.
class HeartRateDevice : public Device {
public:

	// initialize connection with sensor
	HeartRateDevice( byte pin ) : Device( pin ) {
		if (pin == 2)
			attachInterrupt( 0, instHeartRate0, RISING );
		else if (pin == 3)
			attachInterrupt( 1, instHeartRate1, RISING );
		else
			sendError( MEC_INVALID_PIN );
	}

	float value() {
		return _pin == 2 ? _currentHeartRate0 : _currentHeartRate1;
	}
};


//============================================
// PULSE SENSOR
//============================================


// this code is adapted from the code for Pulse Sensor Amped by Joel Murphy and Yury Gitman
#ifdef USE_PULSE_SENSOR_DEVICE


volatile int pulsePin;
volatile int BPM;                      // used to hold the pulse rate
volatile int Signal;                   // holds the incoming raw data
volatile int IBI;                      // holds the time between beats, must be seeded!
volatile boolean Pulse;                // true when pulse wave is high, false when it's low
volatile boolean QS;                   // becomes true when Arduoino finds a beat.
volatile int rate[10];                 // array to hold last ten IBI values
volatile unsigned long sampleCounter;  // used to determine pulse timing
volatile unsigned long lastBeatTime;   // used to find IBI
volatile int P;                        // used to find peak in pulse wave, seeded
volatile int T;                        // used to find trough in pulse wave, seeded
volatile int thresh;                   // used to find instant moment of heart beat, seeded
volatile int amp;                      // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat;            // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat;           // used to seed rate array so we startup with reasonable BPM


void interruptSetup(){
	// Initializes Timer2 to throw an interrupt every 2mS.
	TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
	TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER
	OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
	TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
	sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED
}


// This device is designed to work with the Pulse Sensor Amped hardware.
class PulseSensorDevice : public Device {
public:

	// initialize connection with sensor
	PulseSensorDevice( byte pin ) : Device( pin ) {
		pulsePin = pin;
		IBI = 600;
		Pulse = false;
		QS = false;
		sampleCounter = 0;
		lastBeatTime = 0;
		P = 512;
		T = 512;
		thresh = 512;
		amp = 100;
		firstBeat = true;
		secondBeat = false;
		BPM = 0;
		_initDone = false;
	}

	float value() {
		if(_initDone == false){
			interruptSetup();
			_initDone = true;
		}
		if(QS){
			QS = false;
			return BPM;
		}
		return 0.0f;
	}

private:
	boolean _initDone;
};


// Interrupt using static pointer
ISR(TIMER2_COMPA_vect){

	cli();                                      // disable interrupts while we do this
	Signal = analogRead(pulsePin);              // read the Pulse Sensor
	sampleCounter += 2;                         // keep track of the time in mS with this variable
	int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

	// find the peak and trough of the pulse wave
	if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
		if (Signal < T){                        // T is the trough
			T = Signal;                         // keep track of lowest point in pulse wave
		}
	}

	if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
		P = Signal;                             // P is the peak
	}                                           // keep track of highest point in pulse wave

	// NOW IT'S TIME TO LOOK FOR THE HEART BEAT
	// signal surges up in value every time there is a pulse
	if (N > 250){                                   // avoid high frequency noise
		if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){
			Pulse = true;                               // set the Pulse flag when we think there is a pulse
			IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
			lastBeatTime = sampleCounter;               // keep track of time for next pulse

			if (secondBeat) {                        // if this is the second beat, if secondBeat == TRUE
				secondBeat = false;                  // clear secondBeat flag
				for(int i=0; i<=9; i++){             // seed the running total to get a realisitic BPM at startup
					rate[i] = IBI;
				}
			}

			if (firstBeat) {                         // if it's the first time we found a beat, if firstBeat == TRUE
				firstBeat = false;                   // clear firstBeat flag
				secondBeat = true;                   // set the second beat flag
				sei();                               // enable interrupts again
				return;                              // IBI value is unreliable so discard it
			}


			// keep a running total of the last 10 IBI values
			word runningTotal = 0;                  // clear the runningTotal variable

			for (int i=0; i<=8; i++) {                // shift data in the rate array
				rate[i] = rate[i+1];                  // and drop the oldest IBI value
				runningTotal += rate[i];              // add up the 9 oldest IBI values
			}

			rate[9] = IBI;                          // add the latest IBI to the rate array
			runningTotal += rate[9];                // add the latest IBI to runningTotal
			runningTotal /= 10;                     // average the last 10 IBI values
			BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
			QS = true;                              // set Quantified Self flag
			// QS FLAG IS NOT CLEARED INSIDE THIS ISR
		}
	}

	if (Signal < thresh && Pulse == true) {    // when the values are going down, the beat is over
		Pulse = false;                         // reset the Pulse flag so we can do it again
		amp = P - T;                           // get amplitude of the pulse wave
		thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
		P = thresh;                            // reset these for next time
		T = thresh;
	}

	if (N > 2500) {                            // if 2.5 seconds go by without a beat
		thresh = 512;                          // set thresh default
		P = 512;                               // set P default
		T = 512;                               // set T default
		lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date
		firstBeat = true;                      // set these to avoid noise
		secondBeat = false;                    // when we get the heartbeat back
	}

	sei();                                     // enable interrupts when youre done!

}
#endif


//============================================
// MOISTURE SENSOR
//============================================


// This device takes analog readings on _pin and toggles power via _pin + 1.
class MoistureDevice : public Device {
public:

	// optional argument: delay in milliseconds between powering the sensor and making a reading
	MoistureDevice( byte pin, long delayMillis = 1000 ) : Device( pin ) {
		_delayMillis = delayMillis;
	}

	// optional argument: delay in milliseconds between powering the sensor and making a reading
	MoistureDevice( byte pin, char *args[], int argCount ) : Device( pin ) {
		_delayMillis = 1000;
		if (argCount >= 1) {
			int newDelay = atoi(args[0]);
			if (newDelay > -1) {
				_delayMillis = newDelay;
			}
		}
		init();
	}

	void init() {
		_lastReadTimestamp = 0;
		_lastValue = 0;
		// We get the pin as 0, for instance, but this is A0. So we can use this, defined
		// in pins_arduino.h, to get the real pin number 14, in our example.
		_powerPin = analogInputToDigitalPin( _pin + 1 );
		pinMode( _powerPin, OUTPUT ); // This needs to be analog. Do we just add 13?
	}

	float value() {
		unsigned long now = millis();
		if(_lastReadTimestamp == 0 || now - _lastReadTimestamp > _delayMillis){
			// Turn on the power
			digitalWrite(_powerPin, HIGH);
			// delay(100); // Delaying here provides lower values overall.
			// Read the analog value
			_lastValue = analogRead(_pin);
			// Turn off the power
			digitalWrite(_powerPin, LOW);
			_lastReadTimestamp = now;
		}
		return _lastValue;
	}

private:

	int _powerPin;
	float _lastValue;
	unsigned int _delayMillis;
	unsigned long _lastReadTimestamp;
};


//============================================
// DEVICE FACTORY
//============================================


// create a device sub-class instance based on specified type
Device *createDevice( byte type, byte pin, char *args[], int argCount ) {
	Device *d = NULL;
	switch (type) {
	case DEVICE_TYPE_ANALOG_INPUT:
		d = new AnalogInputDevice( pin, args, argCount );
		break;
	case DEVICE_TYPE_BINARY_INPUT:
		d = new BinaryInputDevice( pin, args, argCount );
		break;
	case DEVICE_TYPE_BINARY_OUTPUT:
		d = new BinaryOutputDevice( pin, args, argCount );
		break;
	case DEVICE_TYPE_PWM:
		d = new PulseWidthDevice( pin );
		break;
	case DEVICE_TYPE_ANALOG_TEMP:
		d = new AnalogTemperatureDevice( pin, args, argCount );
		break;
// You can't use the tone device with the pulse sensor device. They both use timer2
#ifndef USE_PULSE_SENSOR_DEVICE
	case DEVICE_TYPE_TONE:
		d = new ToneDevice( pin );
		break;
#endif
	case DEVICE_TYPE_ATLAS_TEMP:
		d = new AtlasTemperatureDevice();
		break;
	case DEVICE_TYPE_MAX6675:
		d = new MAX6675Device( pin );
		break;
	case DEVICE_TYPE_PULSE_COUNTER:
		d = new PulseCounterDevice( pin );
		break;
	case DEVICE_TYPE_ANALOG_AVERAGE:
		d = new AnalogAverageDevice( pin );
		break;
	case DEVICE_TYPE_BUTTON_SET:
		d = new ButtonSetDevice( pin, args, argCount );
		break;
	case DEVICE_TYPE_CHAINABLE_RGB_LED:
		d = new ChainableRgbLedDevice( pin );
		break;
	case DEVICE_TYPE_PING:
		d = new PingDevice( pin, args, argCount );
		break;
	case DEVICE_TYPE_HEARTRATE:
		d = new HeartRateDevice( pin );
		break;
	case DEVICE_TYPE_MOISTURE:
		d = new MoistureDevice( pin, args, argCount );
		break;

// devices that require USE_X to be defined
#ifdef USE_PULSE_SENSOR_DEVICE
	case DEVICE_TYPE_PULSE_SENSOR:
		d = new PulseSensorDevice( pin );
		break;
#endif
#ifdef USE_GROVE_MOTOR_DEVICE
	case DEVICE_TYPE_MOTOR_DRIVER:
		d = new MotorDriverDevice();
		break;
#endif
#ifdef USE_SERVO_DEVICE
	case DEVICE_TYPE_SERVO:
		d = new ServoDevice( pin );
		break;
#endif
#ifdef USE_DS18B20_DEVICE
	case DEVICE_TYPE_DS18B20:
		d = new DSDevice( pin );
		break;
#endif
#ifdef USE_DHT_DEVICE
	case DEVICE_TYPE_DHT11:
	case DEVICE_TYPE_DHT21:
	case DEVICE_TYPE_DHT22:
	case DEVICE_TYPE_AM2301:
		d = new DHTDevice( pin, type );
		break;
#endif
#ifdef USE_HIH6130_DEVICE
	case DEVICE_TYPE_HIH6130:
		d = new HIH6130Device();
		break;
#endif
#ifdef USE_TSL2561_DEVICE
	case DEVICE_TYPE_TSL2561:
		d = new TSLDevice( args, argCount );
		break;
#endif
#ifdef USE_GROVE_DUST_DEVICE
	case DEVICE_TYPE_GROVE_DUST:
		d = new GroveDustDevice( pin );
		break;
#endif
#ifdef USE_BMP085_DEVICE
	case DEVICE_TYPE_BMP085:
		d = new BMP085Device( args, argCount );
		break;
#endif
#ifdef USE_BMP180_DEVICE
	case DEVICE_TYPE_BMP180:
		d = new BMP180Device( args, argCount );
		break;
#endif
#ifdef USE_WIND_DEVICES
	case DEVICE_TYPE_WIND_DIRECTION:
		d = new WindDirectionDevice( pin );
		break;
	case DEVICE_TYPE_WIND_SPEED:
		d = new WindSpeedDevice( pin );
		break;
#endif
#ifdef USE_RAIN_GAUGE_DEVICE
	case DEVICE_TYPE_RAIN_GAUGE:
		d = new RainGaugeDevice( pin );
		break;
#endif
#ifdef USE_GROVE_COLOR_DEVICE
	case DEVICE_TYPE_GROVE_COLOR:
		d = new GroveColorDevice( args, argCount );
		break;
#endif
#ifdef USE_ATLAS_SERIAL_DEVICE
	case DEVICE_TYPE_ATLAS_SERIAL:
		d = new AtlasSerialDevice( pin );
		break;
	case DEVICE_TYPE_ATLAS_EC:
		d = new AtlasECDevice( pin );
		break;
	case DEVICE_TYPE_ATLAS_DO:
		d = new AtlasDODevice( pin );
		break;
	case DEVICE_TYPE_ATLAS_SERIAL_COLOR:
		d = new AtlasSerialColorDevice( pin );
		break;
#endif
#ifdef USE_MAXBOTIX_SERIAL_DEVICE
	case DEVICE_TYPE_MAXBOTIX_SERIAL_RANGE:
		d = new MaxBotixSerialRangeDevice( pin );
		break;
#endif
#ifdef USE_ADXL345_DEVICE
	case DEVICE_TYPE_ADXL345:
		d = new ADXL345Device( args, argCount );
		break;
#endif
#ifdef USE_HMC5883L_DEVICE
	case DEVICE_TYPE_HMC5883L:
		d = new HMC5883LDevice( args, argCount );
		break;
#endif
#ifdef USE_IR_ENCODER_DEVICE
	case DEVICE_TYPE_IR_ENCODER:
		d = new IREncoderDevice( pin, args, argCount);
		break;
#endif
#ifdef USE_INA219_DEVICE
	case DEVICE_TYPE_INA219:
		d = new INA219Device( args, argCount );
		break;
#endif
#ifdef USE_GROVE_IR_TEMP_DEVICE
	case DEVICE_TYPE_GROVE_IR_TEMP:
		d = new IRTempDevice( pin, args, argCount );
		break;
#endif
#ifdef USE_MINI_PH_DEVICE
	case DEVICE_TYPE_MINI_PH:
		d = new MiniPhDevice( pin );
		break;
#endif
#ifdef USE_GPS_DEVICE
	case DEVICE_TYPE_GPS:
		d = new GPSDevice( pin );
		break;
#endif
#ifdef USE_RF_DEVICE
	case DEVICE_TYPE_RF_TRANSMITTER:
		d = new RFTransmitterDevice( pin, args, argCount );
		break;
	case DEVICE_TYPE_RF_RECEIVER:
		d = new RFReceiverDevice( pin, args, argCount );
		break;
#endif
	default:
		sendError( MEC_INVALID_DEVICE_TYPE );
	}
	if (d)
		d->setType( type );
	return d;
}


#endif // _WIREGARDEN_DEVICES_H_
