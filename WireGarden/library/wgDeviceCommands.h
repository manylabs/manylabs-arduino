// Manylabs WireGarden Arduino Library 0.4.0
// some code adapted from outside sources
// other code copyright ManyLabs 2011-2014
// dual license:
// (1) MIT license (without external GPL'd libraries)
// (2) GPL license (with external GPL'd libraries)
#ifndef _WIREGARDEN_DEVICE_COMMANDS_H_
#define _WIREGARDEN_DEVICE_COMMANDS_H_
#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "wgDevices.h"


// the maximum number of devices can be defined in a sketch; we provide a default here in case not defined in sketch
#ifndef MAX_DEVICES
#define MAX_DEVICES 10
#endif


// array of currently defined devices
Device *g_devices[ MAX_DEVICES ];
unsigned char g_deviceCount = 0;


// current timing state
long g_lastSendTime = 0;
long g_sendInterval = 1000; // default send interval is 1 second


//============================================
// DEVICE SET FUNCTIONS
//============================================


// remove all current device definitions
void resetDevices() {
	for (int i = 0; i < g_deviceCount; i++) {
		delete g_devices[ i ];
	}
	g_deviceCount = 0;
}


// initializes device module
void initDevices() {
	resetDevices();
	for (int i = 0; i < MAX_DEVICES; i++) {
		g_devices[ i ] = NULL;
	}
}


// find the index of a device with the given type/pin in the array of devices;
// returns -1 if not found
int findDeviceIndex( unsigned char type, unsigned char pin ) {
	int deviceIndex = -1;
	for (int i = 0; i < g_deviceCount; i++) {
		Device *d = g_devices[ i ];
		if (d->type() == type && d->pin() == pin) {
			deviceIndex = i;
			break;
		}
	}
	return deviceIndex;
}


// find a device with the given type/pin in the array of devices;
// returns NULL if not found
Device *findDevice( unsigned char type, unsigned char pin ) {
	Device *device = NULL;
	int deviceIndex = findDeviceIndex( type, pin );
	if (deviceIndex >= 0) {
		device = g_devices[ deviceIndex ];
	}
	return device;
}


// send the value(s) of the given device
void sendDeviceValues( Device &device ) {
	byte valueCount = device.valueCount();
	if (valueCount == 1) {
		sendFloat( device.value(), device.decimalPlaces() );
	} else {
		float *values = device.values();
		byte decimalPlaces = device.decimalPlaces();
		for (byte i = 0; i < valueCount; i++) {
			if (i)
				sendString( "," );
			sendFloat( values[ i ], decimalPlaces );
		}
	}
}


// if enough time has elapsed since last send, send device values
void checkSendDevices() {

	// check all currently defined devices (allows devices to make quick check with hardware)
	for (int i = 0; i < g_deviceCount; i++) {
		g_devices[ i ]->update();
	}

	// if enough time has elapsed, send values
	if (g_sendInterval) {
		long currentTime = millis(); // fix: should handle roll-over
		if (currentTime - g_lastSendTime > g_sendInterval) {

			// loop over devices, updating and sending each one
			int sendCount = 0;
			for (byte i = 0; i < g_deviceCount; i++) {
				Device *device = g_devices[ i ];
				if (sendCount)
					sendString( "," );
				else
					sendString( "v:" );
				sendDeviceValues( *device );
				sendCount++;
			}
			if (sendCount)
				sendNewLine();
			g_lastSendTime = currentTime;
		}
	}
}


// send a comma-separated list of device types that are currently enabled
// fix(clean): move this to wgDevices?
void sendEnabledDeviceTypes() {
	char separator = ',';
	sendInt( DEVICE_TYPE_BINARY_INPUT );
	sendChar( separator );
	sendInt( DEVICE_TYPE_ANALOG_INPUT );
	sendChar( separator );
	sendInt( DEVICE_TYPE_BUTTON_SET );
	sendChar( separator );
	sendInt( DEVICE_TYPE_ANALOG_AVERAGE );
	sendChar( separator );
	sendInt( DEVICE_TYPE_ANALOG_TEMP );
	sendChar( separator );
	sendInt( DEVICE_TYPE_PULSE_COUNTER );
	sendChar( separator );
	sendInt( DEVICE_TYPE_MOISTURE );
	sendChar( separator );
#ifdef USE_DHT_DEVICE
	sendInt( DEVICE_TYPE_DHT11 );
	sendChar( separator );
	sendInt( DEVICE_TYPE_DHT21 );
	sendChar( separator );
	sendInt( DEVICE_TYPE_DHT22 );
	sendChar( separator );
	sendInt( DEVICE_TYPE_AM2301 );
	sendChar( separator );
#endif
#ifdef USE_DS18B20_DEVICE
	sendInt( DEVICE_TYPE_DS18B20 );
	sendChar( separator );
#endif
	sendInt( DEVICE_TYPE_ATLAS_TEMP );
	sendChar( separator );
	sendInt( DEVICE_TYPE_MAX6675 );
	sendChar( separator );
#ifdef USE_TSL2561_DEVICE
	sendInt( DEVICE_TYPE_TSL2561 );
	sendChar( separator );
#endif
#ifdef USE_GROVE_COLOR_DEVICE
	sendInt( DEVICE_TYPE_GROVE_COLOR );
	sendChar( separator );
#endif
#ifdef USE_ATLAS_SERIAL_DEVICE
	sendInt( DEVICE_TYPE_ATLAS_SERIAL_COLOR );
	sendChar( separator );
	sendInt( DEVICE_TYPE_ATLAS_SERIAL );
	sendChar( separator );
	sendInt( DEVICE_TYPE_ATLAS_EC );
	sendChar( separator );
	sendInt( DEVICE_TYPE_ATLAS_DO );
	sendChar( separator );
#endif
#ifdef USE_MAXBOTIX_SERIAL_DEVICE
	sendInt( DEVICE_TYPE_MAXBOTIX_SERIAL_RANGE );
	sendChar( separator );
#endif
	sendInt( DEVICE_TYPE_PING );
	sendChar( separator );
#ifdef USE_GROVE_DUST_DEVICE
	sendInt( DEVICE_TYPE_GROVE_DUST );
	sendChar( separator );
#endif
#ifdef USE_BMP085_DEVICE
	sendInt( DEVICE_TYPE_BMP085 );
	sendChar( separator );
#endif
	sendInt( DEVICE_TYPE_HEARTRATE );
	sendChar( separator );
#ifdef USE_HMC5883L_DEVICE
	sendInt( DEVICE_TYPE_HMC5883L );
	sendChar( separator );
#endif
#ifdef USE_ADXL345_DEVICE
	sendInt( DEVICE_TYPE_ADXL345 );
	sendChar( separator );
#endif
#ifdef USE_GROVE_IR_TEMP_DEVICE
	sendInt( DEVICE_TYPE_GROVE_IR_TEMP );
	sendChar( separator );
#endif
#ifdef USE_INA219_DEVICE
	sendInt( DEVICE_TYPE_INA219 );
	sendChar( separator );
#endif
#ifdef USE_IR_ENCODER_DEVICE
	sendInt( DEVICE_TYPE_IR_ENCODER );
	sendChar( separator );
#endif
#ifdef USE_PULSE_SENSOR_DEVICE
	sendInt( DEVICE_TYPE_PULSE_SENSOR );
	sendChar( separator );
#endif
#ifdef USE_WIND_DEVICES
	sendInt( DEVICE_TYPE_WIND_SPEED );
	sendChar( separator );
	sendInt( DEVICE_TYPE_WIND_DIRECTION );
	sendChar( separator );
#endif
#ifdef USE_RAIN_GAUGE_DEVICE
	sendInt( DEVICE_TYPE_RAIN_GAUGE );
	sendChar( separator );
#endif
#ifdef USE_MINI_PH_DEVICE
	sendInt( DEVICE_TYPE_MINI_PH );
	sendChar( separator );
#endif
#ifdef USE_RF_DEVICE
	sendInt( DEVICE_TYPE_RF_RECEIVER );
	sendChar( separator );
#endif
	sendInt( DEVICE_TYPE_BINARY_OUTPUT );
	sendChar( separator );
	sendInt( DEVICE_TYPE_PWM );
	sendChar( separator );
	sendInt( DEVICE_TYPE_TONE );
	sendChar( separator );
	sendInt( DEVICE_TYPE_CHAINABLE_RGB_LED );
	sendChar( separator );
#ifdef USE_SERVO_DEVICE
	sendInt( DEVICE_TYPE_SERVO );
	sendChar( separator );
#endif
#ifdef USE_GROVE_MOTOR_DEVICE
	sendInt( DEVICE_TYPE_MOTOR_DRIVER );
	sendChar( separator );
#endif
#ifdef USE_RF_DEVICE
	sendInt( DEVICE_TYPE_RF_TRANSMITTER );
	sendChar( separator );
#endif
	sendInt( 0 ); // so that we don't have a hanging comma
}


// check for device commands; returns true if valid device command was recognized and executed (whether successful or not)
bool deviceCommand( const char *command, unsigned char argCount, char *args[] ) {
	bool used = true;

	// set device value(s); check this first, because we assume it will be most common
	if (strEq( command, "s" ) && argCount >= 3) {
		int type = atoi( args[ 0 ] );
		int pin = atoi( args[ 1 ] );
		Device *device = findDevice( type, pin );
		// If we didn't find the device, it may be on an analog pin
		// Todo: Find a better way to handle this. This could would conflict, for instance, if you
		// had two binary outputs - one on D2 the other on A2.
		if (!device){
			device = findDevice( type, analogInputToDigitalPin( pin ) );
		}
		if (device) {
			if (argCount == 3) {
				device->setValue( atof( args[ 2 ] ) );
			} else if (argCount == 2 + device->valueCount()) {
				float values[ 10 ];
				for (byte i = 2; i < argCount; i++) {
					values[ i - 2 ] = atof( args[ i ] );
				}
				device->setValues( values );
			} else {
				sendError( MEC_INVALID_VALUES );
			}
		} else {
			sendError( MEC_DEVICE_NOT_FOUND );
		}

	// get device value(s)
	} else if (strEq( command, "g" ) && argCount == 2) {
		int type = atoi( args[ 0 ] );
		int pin = atoi( args[ 1 ] );
		Device *device = findDevice( type, pin );
		if (device) {
			sendString( "dv:" );
			sendDeviceValues( *device );
			sendNewLine();
		} else {
			sendError( MEC_DEVICE_NOT_FOUND );
		}

	// create device; command will replace device with same pin and type
	// (cannot be used to change the type of a device on a particular pin)
	} else if (strEq( command, "d" ) && argCount >= 2) {
		int type = atoi( args[ 0 ] );
		int pin = atoi( args[ 1 ] );

		// process arguments
		char *extraArgs[ 10 ];
		int extraArgCount = 0;
		if (argCount > 2) {
			for (int i = 2; i < argCount; i++) {
				if (extraArgCount < 10)
					extraArgs[ extraArgCount++ ] = args[ i ];
			}
		}

		// if device exists, overwrite it
		int index = findDeviceIndex( type, pin );
		if (index >= 0) {
			Device *device = createDevice( type, pin, extraArgs, extraArgCount );
			if (device) {
				delete g_devices[ index ]; // fix(later): we could try to delete first, but then might end up with gap in device list
				g_devices[ index ] = device;
			}

		// otherwise, add new device
		} else if (g_deviceCount < MAX_DEVICES) {
			Device *device = createDevice( type, pin, extraArgs, extraArgCount );
			if (device) {
				g_devices[ g_deviceCount ] = device;
				g_deviceCount++;
			}
		} else {
			sendError(MEC_DEVICE_LIMIT);
		}

	// check the set of devices that is enabled
	} else if (strEq( command, "dev" ) && argCount == 0) {
		sendString( "i:" );
		sendEnabledDeviceTypes();
		sendNewLine();

	// set the frequency of sending device values
	} else if (strEq( command, "si" ) && argCount == 1) {
		g_sendInterval = atoi( args[ 0 ] );
	} else {
		used = false;
	}
	return used;
}


#endif // _WIREGARDEN_DEVICE_COMMANDS_H_
