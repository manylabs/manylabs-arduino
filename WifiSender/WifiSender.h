// Manylabs WifiSender Library 0.4.0
// copyright Manylabs 2014; MIT license
// --------
// This library provides a simple interface for sending data to a server via WiFi.
// It depends on the WiFly library from Seeed Studio.
#ifndef _MANYLABS_WIFI_SENDER_H_
#define _MANYLABS_WIFI_SENDER_H_
#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "WiFly.h"
#include "HTTPClient.h"


// to specify a different URL, just define a WIFI_POST_URL prior to including this header
#ifndef WIFI_POST_URL
#define WIFI_POST_URL "http://www.manylabs.org/data/rpc/appendData/"
#endif

//============================================
// WIFI SENDER CLASS DEFINITION
//============================================


// The WifiSender class can be used to send HTTP POST messages via WiFi.
class WifiSender {
public:

	// create a new WifiSender object using the given serial object; does not connect until init(); diagStream is for displaying diagnostics
	WifiSender( Stream &serialStream, Stream *diagStream );

	// set network info and parameter buffer (assumes these remain valid for lifetime of object); init wifi; returns false on error
	bool init( const char *networkName, const char *networkPassword, char *parameterBuffer, int parameterBufferLength );

	// add a value to transmit with the next call to send()
	// the template allows these functions to accept const char * or
	// const FlashStringHelper *
	// this means you can call add("this") or add( F("this") )
	template <typename T> void add( const T *value );
	template <typename T> void add( const T *name, const T *value );
	template <typename T> void add( const T *name, float value, byte decimalPlaces = 2 );
	template <typename T> void add( const T *name, double value, byte decimalPlaces = 2 );
	template <typename T> void add( const T *name, int value );
	template <typename T> void add( const T *name, long value );
	template <typename T> void add( const T *name, unsigned long value );

	// post to the server with the values specified since the last call to send(); and with the specified
	// headers; returns false on error
	bool send( const char *headers="Content-Type: text/plain\r\n" );

	// connect to network specified during init
	void join();

	// reboot module
	void reboot();

private:

	// add a string to the parameter buffer
	void append( const char *str );
	void append(const __FlashStringHelper *str);

	// the (externally provided) buffer for POST parameters
	char *m_paramBuf;

	// the length of the buffer
	int m_paramBufLen;

	// the current position in the buffer (next data will be written here)
	int m_paramBufPos;

	// number of parameters currently in param buf
	int m_paramCount;

	// network info
	const char *m_networkName;
	const char *m_networkPassword;

	// true if successfully joined network
	bool m_joined;

	// stream for diagnostic output
	Stream *m_diagStream;

	// the WiFly objects
	WiFly m_wifly;
	HTTPClient m_http;

	unsigned int m_rebootCount;
};


//============================================
// WIFI SENDER IMPLEMENTATION
//============================================


// create a new WifiSender object using the given serial object; does not connect until init(); diagStream is for displaying diagnostics
WifiSender::WifiSender( Stream &serialStream, Stream *diagStream = NULL ) : m_wifly( serialStream ), m_diagStream( diagStream ) {
	m_paramBuf = NULL;
	m_paramBufLen = 0;
	m_paramBufPos = 0;
	m_paramCount = 0;
	m_networkName = NULL;
	m_networkPassword = NULL;
	m_joined = false;
	m_rebootCount = 0;
}


// set network info and parameter buffer (assumes these remain valid for lifetime of object); init wifi; returns false on error
bool WifiSender::init( const char *networkName, const char *networkPassword, char *parameterBuffer, int parameterBufferSize ) {
	m_paramBuf = parameterBuffer;
	m_paramBufLen = parameterBufferSize;
	m_paramBufPos = 0;
	m_paramCount = 0;
	m_networkName = networkName;
	m_networkPassword = networkPassword;
	m_joined = false;

	// init wifly library; don't try to join until send
	m_wifly.reset();
	if (m_wifly.init() == false) {
		return false;
	}
	return true;
}


// connect to network specified during init
void WifiSender::join() {
	m_joined = false;
	if (m_wifly.join( m_networkName, m_networkPassword, WIFLY_AUTH_WPA2_PSK ) == false) {
		if( m_diagStream ) m_diagStream->println( F("unable to join") );
		return;
	}
	if (m_wifly.isAssociated() == false) {
		if( m_diagStream ) m_diagStream->println( F("not associated after join") );
		return;
	}
	m_wifly.sendCommand( "set comm remote 0\r" ); // disable *HELLO* message at start of each post
	m_joined = true;
}

// reboot module
void WifiSender::reboot() {
	m_wifly.reboot();
	m_joined = false;
	return;
}


// add data to transmit with the next call to send
template <typename T>
void WifiSender::add(const T *value)
{
    append(value);
}


// add a value to transmit with the next call to send()
template <typename T>
void WifiSender::add( const T *name, const T *value ) {
	if (m_paramCount)
		append( F("&") );
	append( name );
	append( F("=") );
	append( value );
	m_paramCount++;
}


// add a value to transmit with the next call to send()
template <typename T>
void WifiSender::add( const T *name, float value, byte decimalPlaces ) {
	add( name, (double) value, decimalPlaces );
}


// add a value to transmit with the next call to send()
template <typename T>
void WifiSender::add( const T *name, double value, byte decimalPlaces ) {
	if (m_paramBufPos + 15 < m_paramBufLen) { // we'll assume that the value doesn't have more than 14 digits
		if (m_paramCount)
			append( F("&") );
		append( name );
		append( F("=") );
		dtostrf( value, decimalPlaces, decimalPlaces, m_paramBuf + m_paramBufPos );
		while (m_paramBuf[ m_paramBufPos ] && m_paramBufPos < m_paramBufLen) // find new end of string
			m_paramBufPos++;
		m_paramCount++;
	}
}


// add a value to transmit with the next call to send()
template <typename T>
void WifiSender::add( const T *name, int value ) {
	add( name, (long) value );
}


// add a value to transmit with the next call to send()
template <typename T>
void WifiSender::add( const T *name, long value ) {
	if (m_paramBufPos + 12 < m_paramBufLen) {
		if (m_paramCount)
			append( F("&") );
		append( name );
		append( F("=") );
		ltoa( value, m_paramBuf + m_paramBufPos, 10 );
		while (m_paramBuf[ m_paramBufPos ] && m_paramBufPos < m_paramBufLen) // find new end of string
			m_paramBufPos++;
		m_paramCount++;
	}
}


// add a value to transmit with the next call to send()
template <typename T>
void WifiSender::add( const T *name, unsigned long value ) {
	if (m_paramBufPos + 12 < m_paramBufLen) {
		if (m_paramCount)
			append( F("&") );
		append( name );
		append( F("=") );
		ultoa( value, m_paramBuf + m_paramBufPos, 10 );
		while (m_paramBuf[ m_paramBufPos ] && m_paramBufPos < m_paramBufLen) // find new end of string
			m_paramBufPos++;
		m_paramCount++;
	}
}

// post to the server with the values specified since the last call to send(); returns false on error
bool WifiSender::send(const char *headers) {
	bool success = false;

	// attempt to join network if not done already
	if (m_joined == false) {
		join();
	}

	// if connected, do the HTTP POST
	if (m_wifly.isAssociated()) {
		if( m_diagStream ) {
			m_diagStream->println( F("POST:") );
			m_diagStream->println( m_paramBuf );
		}
		int errCode = m_http.post( WIFI_POST_URL, headers, m_paramBuf );
		if (errCode) {
			if( m_diagStream ) {
				m_diagStream->print( F("error:") );
				m_diagStream->println( errCode );
			}
			m_joined = false; // could be network error; try reconnecting next time
			if(errCode == -2){
				if( m_diagStream ) {
					m_diagStream->print( F("rebooting: ") );
					m_diagStream->println( m_rebootCount++ );
				}
				reboot();
			}
		} else {
			success = true;
			if( m_diagStream ) {
				char get;
				while (m_wifly.receive((uint8_t *)&get, 1, 1000) == 1) {
					m_diagStream->print(get);
				}
			}
		}
	}else{
		m_joined = false;
	}

	// clear buffer for next round
	m_paramBuf[ 0 ] = 0;
	m_paramBufPos = 0;
	m_paramCount = 0;
	return success;
}


// add a string to the parameter buffer
void WifiSender::append( const char *str ) {
	while (str[ 0 ]) {
		m_paramBuf[ m_paramBufPos++ ] = str[ 0 ];
		str++;
		if (m_paramBufPos + 1 >= m_paramBufLen) // leave room for zero terminator
			break;
	}
	m_paramBuf[ m_paramBufPos ] = 0; // add zero terminator
}

// add a flash string (using the F() macro) to the parameter buffer
void WifiSender::append( const __FlashStringHelper *str ){
	PGM_P p = reinterpret_cast<PGM_P>(str);
	char c = pgm_read_byte(p++);
	while (c) {
		m_paramBuf[ m_paramBufPos++ ] = c;
		c = pgm_read_byte(p++);
		if (m_paramBufPos + 1 >= m_paramBufLen) // leave room for zero terminator
			break;
	}
	m_paramBuf[ m_paramBufPos ] = 0; // add zero terminator
}


#endif // _MANYLABS_WIFI_SENDER_H_
