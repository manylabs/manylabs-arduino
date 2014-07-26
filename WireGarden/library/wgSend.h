// Manylabs WireGarden Arduino Library
// some code adapted from outside sources
// other code copyright ManyLabs 2011-2014
// dual license:
// (1) MIT license (without external GPL'd libraries)
// (2) GPL license (with external GPL'd libraries)
#ifndef _WIREGARDEN_SEND_H_
#define _WIREGARDEN_SEND_H_
#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


// if defined we allow the use of a callback to send out text/serial data
#define USE_CUSTOM_SENDER


// if true, will send data to normal serial port
bool g_enableSerial = true;


// a callback for sending data to other control devices
void (*g_sender)( byte *data, int size ) = NULL;


// set a callback for sending data to other control devices
void registerSender( void (*sender)( byte *data, int size ) ) {
	g_sender = sender;
}


// send a string to the control device
void sendString( const char *str ) {
	if (g_enableSerial)
		Serial.print( str );
#ifdef USE_CUSTOM_SENDER
	if (g_sender) {
		g_sender( (byte *) str, strlen( str ) );
	}
#endif // USE_CUSTOM_SENDER
}


// send a character to the control device
void sendChar( char c ) {
	if (g_enableSerial)
		Serial.print( c );
#ifdef USE_CUSTOM_SENDER
	if (g_sender) {
		g_sender( (byte *) &c, 1 );
	}
#endif // USE_CUSTOM_SENDER
}


// send an integer to the control device
void sendInt( int value ) {
	if (g_enableSerial)
		Serial.print( value, DEC );
#ifdef USE_CUSTOM_SENDER
	if (g_sender) {
		char convBuf[ 10 ];
		itoa( value, convBuf, 10 );
		g_sender( (byte *) convBuf, strlen( convBuf ) );
	}
#endif // USE_CUSTOM_SENDER
}


// send a floating-point number to the control device
void sendFloat( float value, int decimalPlaces ) {
	if (g_enableSerial)
		Serial.print( value, decimalPlaces );
#ifdef USE_CUSTOM_SENDER
	if (g_sender) {
		char convBuf[ 15 ];
		dtostrf( value, decimalPlaces, decimalPlaces, convBuf );
		g_sender( (byte *) convBuf, strlen( convBuf ) );
	}
#endif // USE_CUSTOM_SENDER
}


// send a newline (end-of-line character)
void sendNewLine() {
	if (g_enableSerial)
		Serial.println();
#ifdef USE_CUSTOM_SENDER
	if (g_sender) {
		char c = 10; // fix: do we want 10 or 13 or both?
		g_sender( (byte *) &c, 1 );
	}
#endif // USE_CUSTOM_SENDER
}


// send an error code; the error must be a string initialized with the F() macro
// for instance: F("error: some error (0)")
void sendError( const __FlashStringHelper *ifsh ){
	PGM_P p = reinterpret_cast<PGM_P>( ifsh );
	while (true) {
		unsigned char c = pgm_read_byte( p++ );
		if (c == 0) 
			break;
		sendChar( c );
	}
	sendNewLine();
}


#endif // _WIREGARDEN_SEND_H_
