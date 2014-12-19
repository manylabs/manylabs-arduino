// Manylabs GprsSender Library 0.1.0
// copyright Manylabs 2014; MIT license
// --------
// This library provides a simple interface for sending data to a server via
// GPRS. Our setup uses the Adafruit FONA, but much of this code would work the
// same with other setups such as the GPRS Bee.
#ifndef _MANYLABS_GPRS_SENDER_H_
#define _MANYLABS_GPRS_SENDER_H_

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include <avr/wdt.h> // Watchdog timer

#ifdef USE_MANYLABS_DATA_AUTH
#include "ManylabsDataAuth.h"
#endif

// These defines control what server the GprsSender will post to
// If you were trying to post to "www.google.com:1234/post/here/", here's how
// it breaks down:
//
// The Host is the base part of the url: "www.google.com"
//
// The Port is usually 80, but could be different. In the above example the port
// is: 1234. If your url doesn't have a port, just use the default.
//
// The Path is the rest of the url: "/post/here/"
#ifndef GPRS_POST_HOST
    #define GPRS_POST_HOST "www.manylabs.org"
    #define GPRS_POST_PATH "/data/api/v1/appendData/"
#endif
#ifndef GPRS_POST_PORT
    #define GPRS_POST_PORT "80"
#endif

// Length of buffer for replies from the SIM module
#define SIM_BUF_LEN 65

// Timeouts: These control how long the code will wait for different responses.
//
// Default timeout for replies from the SIM module
#define DEFAULT_TIMEOUT_MS 2000

// Default timeout for network operations. (Connecting to the server, waiting
// for a server response)
#define DEFAULT_NETWORK_TIMEOUT_MS 10000

// Default timeout for network registration. (Connecting to the cell network)
#define DEFAULT_NETWORK_REG_TIMEOUT_MS 60000

#define diagStreamPrint(...) { if (m_diagStream && m_useDiagStream) m_diagStream->print(__VA_ARGS__); }
#define diagStreamPrintLn(...) { if (m_diagStream && m_useDiagStream) m_diagStream->println(__VA_ARGS__); }

//============================================
// GPRS SENDER CLASS DEFINITION
//============================================


// The GprsSender class can be used to send HTTP POST messages via GRPS.
class GprsSender {
public:

    // create a new GPRS object using the given serial object
    // diagStream is for displaying diagnostics
    // reset pin is specific to the Adafruit FONA and is used for rebooting
    GprsSender( int resetPin, Stream &serialStream, Stream &diagStream );

    // Same as above but without diagnostics
    GprsSender( int resetPin, Stream &serialStream );

    // set network info and parameter buffer (assumes these remain valid for
    // lifetime of object), reboots the module (specific to the Adafruit FONA)
    // and waits for network registration (up to 60 seconds).
    // returns true on successful network registration, or false on timeout.
    bool init( char *parameterBuffer, int parameterBufferLength,
        const __FlashStringHelper *apn,
        const __FlashStringHelper *apnUsername = 0,
        const __FlashStringHelper *apnPassword = 0 );

    // reboot the SIM module
    void reboot();

    // add a value to transmit with the next call to send()
    // the template allows these functions to accept const char * or
    // const FlashStringHelper *
    // this means you can call add( "this", 0 ) or add( F("this"), 1 )
    template <typename T> void add( const T *value );
    template <typename T> void add( const T *name, const T *value );
    template <typename T> void add( const T *name, float value, byte decimalPlaces = 2 );
    template <typename T> void add( const T *name, double value, byte decimalPlaces = 2 );
    template <typename T> void add( const T *name, int value );
    template <typename T> void add( const T *name, long value );
    template <typename T> void add( const T *name, unsigned long value );

    // post to the server with the values specified since the last call to send
    // returns false on error. you can check the reason for the error with
    // lastErrorCode
    bool send();

    #ifdef USE_MANYLABS_DATA_AUTH

    // same as send, but also includes the Manylabs data authentication header
    // calculated from the provided keys
    bool sendWithManylabsDataAuth( const __FlashStringHelper *public_key,
        const __FlashStringHelper *private_key );

    #endif

    // retrieve the last HTTP status code (assuming the post was successful)
    int lastStatusCode(){ return m_lastStatusCode; }

    // returns an error code for the last failure to send
    // 0 No Error: There was no error. The send was successful
    // 1 GPRS Error: The error was related to the SIM module. If you receive
    //   several of these, a reboot may be helpful.
    // 2 Server / Network Error: The error was related to the cell network, the
    //   server, or other network connection issues. A reboot is unlikely to
    //   help. The server may be unreachable, or perhaps there is no cell
    //   service in your current location.
    int lastErrorCode(){ return m_lastErrorCode; }

    // continually checks, up to the timeout, for successful network
    // registration. depending on network conditions, this can take a while.
    //
    // returns true on success, or false on timeout.
    bool waitForNetworkReg( uint32_t timeout = DEFAULT_NETWORK_REG_TIMEOUT_MS );

    // returns signal strength (RSSI). according to the SIM module information,
    // the values essentially go from 0-31 with higher being better. the value
    // 99 is reserved for "not known" or "not detectable". reproduced here:
    //
    // 0:    -115 dBm or less
    // 1:    -111 dBm
    // 2-30: -110 ... -54 dBm
    // 31:   -52 dBm or greater
    // 99:   not known or not detectable
    int signalStrength( uint32_t timeout = DEFAULT_TIMEOUT_MS );

    // returns true if there is a diagnostic stream and diagnostics are enabled
    // otherwise, returns false
    bool diagnosticsEnabled(){ return m_diagStream && m_useDiagStream; }

    // disable diagnostics. this is useful to temporarily disable diagnostic
    // output if you've created the GprsSender with a diagnostic stream.
    void disableDiagnostics(){ m_useDiagStream = false; }

    // enable diagnostics. use this to re-enable diagnostic output if you've
    // created the GprsSender with a diagnostic stream, but disabled it with
    // disableDiagnostics().
    void enableDiagnostics(){ m_useDiagStream = true; }

    // delay and reset the watchdog timer during the delay. If you're using a
    // watchdog timer to reset your Arduino, you can use this delay method
    // to prevent resetting the Arduino during the delay.
    void delayAndWdtReset( uint32_t ms );

private:

    // send a command to the SIM module
    template <typename T> void sendCommand( const T *command );

    // wait for a reply from the SIM module
    // returns false on timeout
    bool waitForReply( const __FlashStringHelper *Reply,
        uint32_t timeout = DEFAULT_TIMEOUT_MS );

    // wait for the prompt "> " from the SIM module
    // returns false on timeout
    bool waitForPrompt( uint32_t timeout = DEFAULT_TIMEOUT_MS );

    // send a command and then wait for the reply
    // returns false on timeout
    bool sendCommandWaitForReply( const __FlashStringHelper *command,
        const __FlashStringHelper *reply,
        uint32_t timeout = DEFAULT_TIMEOUT_MS );

    // Reads a line formatted <CR><LF>reply<CR><LF> from the SIM module
    // returns the number of bytes read or -1 on timeout
    int readLine( uint32_t timeout = DEFAULT_TIMEOUT_MS );

    // attempts to parse the HTTP status code from the server response
    int readStatusCode( uint32_t timeout = DEFAULT_TIMEOUT_MS );

    // send raw serial data to the SIM module without first flushing the input
    // and without adding '\r' as sendCommand does
    template <typename T> void sendRaw( const T *raw );
    template <typename T> void sendRaw( T raw );

    // start the GPRS connection
    bool startConnection();

    // write the default headers
    void writeDefaultHeaders();

    // write the content-length header and the current data. clear the parameter
    // buffer for the next round
    void writeData();

    // tell the SIM module to send the data
    bool sendData();

    // close the GPRS connection - Sometimes this takes a bit longer, so it
    // defaults to a double timeout
    bool closeConnection( uint32_t timeout = DEFAULT_TIMEOUT_MS * 2 );

    // read from the serial stream until it's empty. All read data will be sent
    // to the debug stream unless printFlushed is false
    void flushInput( bool printFlushed = true );

    // return true if the current millis value has passed the given value that
    // was constructed as: timestamp = millis() + some value
    bool timedOut( uint32_t timestamp );

    // add a string to the parameter buffer
    void append( const char *str );
    void append( const __FlashStringHelper *str );

    // clears the parameter buffer
    void clearParameterBuffer();

    // the (externally provided) buffer for POST parameters
    char *m_paramBuf;

    // the length of the buffer
    int m_paramBufLen;

    // the current position in the buffer (next data will be written here)
    int m_paramBufPos;

    // number of parameters currently in paramBuf
    int m_paramCount;

    // network info
    const __FlashStringHelper *m_apn;
    const __FlashStringHelper *m_apnUsername;
    const __FlashStringHelper *m_apnPassword;

    // stream for communicating with SIM module
    Stream *m_serialStream;

    // stream for diagnostic output
    Stream *m_diagStream;

    // used to disable diagnostics even when we have a diagnostic stream
    bool m_useDiagStream;

    // buffer for responses from the SIM module
    char m_simBuf[SIM_BUF_LEN];

    // current position in the buffer
    int m_simBufPos;

    // the last HTTP status code
    int m_lastStatusCode;

    // the reason for the last failure to send
    int m_lastErrorCode;

    // the pin we should toggle to reset the module (specific to the Adafruit
    // FONA)
    int m_resetPin;
};


//============================================
// GPRS SENDER IMPLEMENTATION
//============================================

// create a new GPRS object using the given serial object
// diagStream is for displaying diagnostics
// reset pin is specific to the Adafruit FONA and is used for rebooting
GprsSender::GprsSender( int resetPin, Stream &serialStream, Stream &diagStream )
    : m_serialStream( &serialStream ), m_diagStream( &diagStream ) {

    m_simBufPos = 0;

    m_paramBuf = NULL;
    m_paramBufLen = 0;
    m_paramBufPos = 0;
    m_paramCount = 0;

    m_lastStatusCode = -1;
    m_resetPin = resetPin;

    m_useDiagStream = true;
}

// Same as above but without diagnostics
GprsSender::GprsSender( int resetPin, Stream &serialStream )
    : m_serialStream( &serialStream ), m_diagStream( NULL ) {

    m_simBufPos = 0;

    m_paramBuf = NULL;
    m_paramBufLen = 0;
    m_paramBufPos = 0;
    m_paramCount = 0;

    m_lastStatusCode = -1;
    m_resetPin = resetPin;

    m_useDiagStream = true;
}

// set network info and parameter buffer (assumes these remain valid for
// lifetime of object), reboots the module (specific to the Adafruit FONA)
// and waits for network registration (up to 60 seconds).
// returns true on successful network registration, or false on timeout.
bool GprsSender::init( char *parameterBuffer, int parameterBufferLength,
    const __FlashStringHelper *apn,
    const __FlashStringHelper *apnUsername,
    const __FlashStringHelper *apnPassword ) {

    m_paramBuf = parameterBuffer;
    m_paramBufLen = parameterBufferLength;
    m_paramBufPos = 0;
    m_paramCount = 0;

    m_apn = apn;
    m_apnUsername = apnUsername;
    m_apnPassword = apnPassword;

    reboot();

    // Wait to attach to the network.
    return waitForNetworkReg();
}

// reboot the SIM module
void GprsSender::reboot() {

    // This first part is specific to the Adafruit FONA:

    // Toggle the reset pin low for 100 ms
    pinMode(m_resetPin, OUTPUT);
    digitalWrite(m_resetPin, HIGH);
    delayAndWdtReset(10);
    digitalWrite(m_resetPin, LOW);
    delayAndWdtReset(100);
    digitalWrite(m_resetPin, HIGH);

    // Give it some time to reboot
    delayAndWdtReset(5000);

    flushInput(false); // Don't print garbage

    // Check a few times for an OK response from the SIM module
    for(int i=0; i<3; i++){
        if(sendCommandWaitForReply(F("AT"), F("OK"))){
            break;
        }
        delayAndWdtReset(100);
    }

    flushInput(false); // Don't print garbage

    // End FONA specific part

    // Clear parameter buffer
    clearParameterBuffer();

    // Disable echoing commands
    sendCommandWaitForReply(F("ATE0"), F("OK"));

    // Show error codes
    sendCommandWaitForReply(F("AT+CMEE=1"), F("OK"));
}

/**
 * Functions for adding and removing data from the parameter buffer
 */

// add data to transmit with the next call to send
template <typename T>
void GprsSender::add(const T *value)
{
    append(value);
}


// add a value to transmit with the next call to send()
template <typename T>
void GprsSender::add( const T *name, const T *value ) {
    if (m_paramCount)
        append( F("&") );
    append( name );
    append( F("=") );
    append( value );
    m_paramCount++;
}


// add a value to transmit with the next call to send()
template <typename T>
void GprsSender::add( const T *name, float value, byte decimalPlaces ) {
    add( name, (double) value, decimalPlaces );
}


// add a value to transmit with the next call to send()
template <typename T>
void GprsSender::add( const T *name, double value, byte decimalPlaces ) {

    // we'll assume that the value doesn't have more than 14 digits
    if (m_paramBufPos + 15 < m_paramBufLen) {
        if (m_paramCount)
            append( F("&") );
        append( name );
        append( F("=") );
        dtostrf( value, decimalPlaces, decimalPlaces,
            m_paramBuf + m_paramBufPos );

        // find new end of string
        while (m_paramBuf[ m_paramBufPos ] && m_paramBufPos < m_paramBufLen){
            wdt_reset();
            m_paramBufPos++;
        }
        m_paramCount++;
    }
}


// add a value to transmit with the next call to send()
template <typename T>
void GprsSender::add( const T *name, int value ) {
    add( name, (long) value );
}


// add a value to transmit with the next call to send()
template <typename T>
void GprsSender::add( const T *name, long value ) {
    if (m_paramBufPos + 12 < m_paramBufLen) {
        if (m_paramCount)
            append( F("&") );
        append( name );
        append( F("=") );
        ltoa( value, m_paramBuf + m_paramBufPos, 10 );

        // find new end of string
        while (m_paramBuf[ m_paramBufPos ] && m_paramBufPos < m_paramBufLen){
            wdt_reset();
            m_paramBufPos++;
        }
        m_paramCount++;
    }
}


// add a value to transmit with the next call to send()
template <typename T>
void GprsSender::add( const T *name, unsigned long value ) {
    if (m_paramBufPos + 12 < m_paramBufLen) {
        if (m_paramCount)
            append( F("&") );
        append( name );
        append( F("=") );
        ultoa( value, m_paramBuf + m_paramBufPos, 10 );

        // find new end of string
        while (m_paramBuf[ m_paramBufPos ] && m_paramBufPos < m_paramBufLen){
            wdt_reset();
            m_paramBufPos++;
        }
        m_paramCount++;
    }
}


// add a string to the parameter buffer
void GprsSender::append( const char *str ) {
    while (str[ 0 ]) {
        wdt_reset();
        m_paramBuf[ m_paramBufPos++ ] = str[ 0 ];
        str++;

        // leave room for zero terminator
        if (m_paramBufPos + 1 >= m_paramBufLen)
            break;
    }

    // add zero terminator
    m_paramBuf[ m_paramBufPos ] = 0;
}

// add a flash string (using the F() macro) to the parameter buffer
void GprsSender::append( const __FlashStringHelper *str ) {
    PGM_P p = reinterpret_cast<PGM_P>(str);
    char c = pgm_read_byte(p++);
    while (c) {
        wdt_reset();
        m_paramBuf[ m_paramBufPos++ ] = c;
        c = pgm_read_byte(p++);

        // leave room for zero terminator
        if (m_paramBufPos + 1 >= m_paramBufLen)
            break;
    }

    // add zero terminator
    m_paramBuf[ m_paramBufPos ] = 0;
}

// clears the parameter buffer
void GprsSender::clearParameterBuffer() {
    m_paramBuf[ 0 ] = 0;
    m_paramBufPos = 0;
    m_paramCount = 0;
}


/**
 * Functions for managing the connection and sending data
 */

// start the GPRS connection
bool GprsSender::startConnection() {

    // Attach to GPRS service (CGATT) - Max response time of 10 sec
    if( !sendCommandWaitForReply(F("AT+CGATT=1"), F("OK"), 10000) ){
        diagStreamPrintLn(F("CGATT Fail - Retrying"));

        // If the SIM module hasn't finished registering with the network, this
        // will fail on the first try. Protect against that here.
        if( !sendCommandWaitForReply(F("AT+CGATT=1"), F("OK"), 10000) ){
            diagStreamPrintLn(F("CGATT Fail"));
            m_lastErrorCode = 1;
            return false;
        }
    }

    // Set credentials (CSTT) - This we need to include the credentials here so
    // we handle this a bit differently than some other commands
    flushInput();
    diagStreamPrint(F("->"));
    sendRaw(F("AT+CSTT=\""));
    sendRaw(m_apn);
    sendRaw(F("\""));
    if(m_apnUsername){
        sendRaw(F(",\""));
        sendRaw(m_apnUsername);
        sendRaw(F("\""));
    }
    if(m_apnPassword){
        sendRaw(F(",\""));
        sendRaw(m_apnPassword);
        sendRaw(F("\""));
    }
    sendRaw(F("\r"));
    diagStreamPrintLn();
    if( !waitForReply(F("OK")) ){
        diagStreamPrintLn(F("CSTT Fail"));
        m_lastErrorCode = 1;
        return false;
    }

    // Start wireless connection (CIICR)
    // Every once in a while this takes quite a bit of time.
    if( !sendCommandWaitForReply(F("AT+CIICR"), F("OK"), DEFAULT_NETWORK_TIMEOUT_MS) ){
        diagStreamPrintLn(F("CIICR Fail"));
        m_lastErrorCode = 1;
        return false;
    }

    // Reset PDP context (CIPSHUT). Otherwise we can sometime get stuck in the
    // "PDP DEACT" state
    if( !closeConnection() ){
        diagStreamPrintLn(F("CIPSHUT Fail"));
        m_lastErrorCode = 1;
        return false;
    }

    // Open the connection to the server (CIPSTART)
    flushInput();
    diagStreamPrint(F("->"));
    sendRaw(F("AT+CIPSTART=\"TCP\",\""));
    sendRaw(F(GPRS_POST_HOST)); // Server
    sendRaw(F("\",\""));
    sendRaw(F(GPRS_POST_PORT)); // Port
    sendRaw(F("\"\r"));
    diagStreamPrintLn();
    if( !waitForReply(F("OK")) ){
        diagStreamPrintLn(F("CIPSTART Fail"));
        m_lastErrorCode = 1;
        return false;
    }

    // We'll get CONNECT OK once the TCP connection is established. This is
    // dependent on the cell network and the server itself.
    if( !waitForReply(F("CONNECT OK"), DEFAULT_NETWORK_TIMEOUT_MS) ){
        diagStreamPrintLn(F("TCP Fail"));
        m_lastErrorCode = 2;
        return false;
    }

    sendCommand(F("AT+CIPSEND"));
    if(!waitForPrompt()){
        diagStreamPrintLn(F("CIPSEND Fail"));
        m_lastErrorCode = 1;
        return false;
    }

    return true;
}

// write the default headers
void GprsSender::writeDefaultHeaders() {
    flushInput();
    diagStreamPrint(F("->"));
    sendRaw(F("POST "));
    sendRaw(F(GPRS_POST_PATH));
    sendRaw(F(" HTTP/1.1\r\n"));
    sendRaw(F("Host: "));
    sendRaw(F(GPRS_POST_HOST));
    sendRaw(F(":"));
    sendRaw(F(GPRS_POST_PORT));
    sendRaw(F("\r\n"));
    sendRaw(F("Connection: close\r\n"));
    sendRaw(F("Content-Type: application/x-www-form-urlencoded\r\n"));
    diagStreamPrintLn();
}

// write the content-length header and the current data. clear the parameter
// buffer for the next round
void GprsSender::writeData() {
    flushInput();
    diagStreamPrint(F("->"));

    // Content Length header
    sendRaw(F("Content-Length: "));
    sendRaw(strlen(m_paramBuf));
    sendRaw(F("\r\n"));

    // Blank line before data
    sendRaw(F("\r\n"));

    // Data
    sendRaw(m_paramBuf);

    // clear buffer for next round
    clearParameterBuffer();
    diagStreamPrintLn();
}

// tell the SIM module to send the data
bool GprsSender::sendData() {

    // Send Ctrl-Z: ((char)26)
    flushInput();
    diagStreamPrint(F("->"));
    sendRaw((char)26);
    diagStreamPrintLn();

    // We'll get "SEND OK" if the data was sent and received by the server.
    // If this was UDP, "SEND OK" would only mean the data was sent. Since it's
    // TCP, that response means the server received it.
    return waitForReply(F("SEND OK"), DEFAULT_NETWORK_TIMEOUT_MS);
}

// close the GPRS connection - Sometimes this takes a bit longer, so it
// defaults to a double timeout
bool GprsSender::closeConnection( uint32_t timeout ) {

    // Close the connection (CIPSHUT)
    return sendCommandWaitForReply(F("AT+CIPSHUT"), F("SHUT OK"), timeout);
}

// post to the server with the values specified since the last call to send
// returns false on error. you can check the reason for the error with
// lastErrorCode
bool GprsSender::send() {
    if(!startConnection()){
        closeConnection();

        // startConnection sets it's own values for m_lastErrorCode, so don't
        // set that here.
        return false;
    }
    writeDefaultHeaders();
    writeData();
    if(!sendData()){
        closeConnection();
        m_lastErrorCode = 2;
        return false;
    }

    // Check for a response
    // The timeout here depends on a lot: Network connection, server, etc.
    m_lastStatusCode = readStatusCode(DEFAULT_NETWORK_TIMEOUT_MS);

    // This last shut often needs a longer timeout for some reason
    if( !closeConnection(10000) ){
        m_lastErrorCode = 1;
        return false;
    }

    m_lastErrorCode = 0;
    return true;
}

#ifdef USE_MANYLABS_DATA_AUTH

// same as send, but also includes the Manylabs data authentication header
// calculated from the provided keys
bool GprsSender::sendWithManylabsDataAuth(const __FlashStringHelper *public_key,
    const __FlashStringHelper *private_key ) {

    if(!startConnection()){
        closeConnection();

        // startConnection sets it's own values for m_lastErrorCode, so don't
        // set that here.
        return false;
    }
    writeDefaultHeaders();

    writeAuthHeader(public_key, private_key, m_paramBuf ,*m_serialStream);

    writeData();
    if(!sendData()){
        m_lastErrorCode = 2;
        closeConnection();
        return false;
    }

    // Check for a response
    // The timeout here depends on a lot: Network connection, server, etc.
    m_lastStatusCode = readStatusCode(DEFAULT_NETWORK_TIMEOUT_MS);

    // This last shut often needs a longer timeout for some reason
    if( !closeConnection(10000) ){
        m_lastErrorCode = 1;
        return false;
    }

    m_lastErrorCode = 0;
    return true;
}

#endif

/**
 * Functions for communicating with the SIM module
 */

// read from the serial stream until it's empty. All read data will be sent
// to the debug stream unless printFlushed is false
void GprsSender::flushInput(bool printFlushed){
    bool printed = false;
    char c;
    while (m_serialStream->available()){
        wdt_reset();
        c = m_serialStream->read();
        printed = true;
        if(printFlushed){
            diagStreamPrint(c);
        }
    }
    if(printed && printFlushed){
        diagStreamPrintLn();
    }
}

// return true if the current millis value has passed the given value that
// was constructed as: timestamp = millis() + some value
bool GprsSender::timedOut(uint32_t timestamp) {
    wdt_reset();

    // If the current value of millis has "passed" the timestamp the value of
    // millis - timestamp will be positive. Otherwise it will be negative.
    // http://playground.arduino.cc/Code/TimingRollover
    return (long)(millis() - timestamp) >= 0;
}

// send a command to the SIM module
template <typename T>
void GprsSender::sendCommand( const T *command ) {
    flushInput();
    m_serialStream->print(command);
    m_serialStream->print("\r");
    diagStreamPrint(F("->"));
    diagStreamPrintLn(command);
}

// send raw serial data to the SIM module without first flushing the input
// and without adding '\r' as sendCommand does
template <typename T>
void GprsSender::sendRaw( const T *raw ) {
    m_serialStream->print(raw);
    diagStreamPrint(raw);
}
template <typename T>
void GprsSender::sendRaw( T raw ) {
    m_serialStream->print(raw);
    diagStreamPrint(raw);
}

// wait for a reply from the SIM module
// returns false on timeout
bool GprsSender::waitForReply( const __FlashStringHelper *reply,
    uint32_t timeout ) {

    // While read until timeout
    diagStreamPrint(F("<-"));
    bool found = false;
    while(readLine(timeout) >= 0){

        // Does the buffer match the reply?
        if( strncmp_P(m_simBuf,
            (const char*)reply, strlen_P((const char*)reply)) == 0 ){
          found = true;
          break;
        }
    }
    diagStreamPrintLn();
    return found;
}

// wait for the prompt "> " from the SIM module
// returns false on timeout
bool GprsSender::waitForPrompt( uint32_t timeout ) {

    // The prompt is a bit different than other replies.
    // It comes as "<CR><LF>> ". (That last part is the ">" character followed
    // by a space).
    diagStreamPrint(F("<-"));
    uint32_t timestamp = millis() + timeout;
    m_simBufPos = 0;
    bool replyStarted = false;
    bool foundBracket = false;
    while(!timedOut(timestamp)){
        while(m_serialStream->available()){
            wdt_reset();
            char c = m_serialStream->read();
            if(c > 31 && c < 128){
                diagStreamPrint(c);
            }else{
                diagStreamPrint(c, HEX);
            }

            if(replyStarted && (m_simBufPos < SIM_BUF_LEN)){
                m_simBuf[m_simBufPos] = c;
                m_simBufPos++;
                if(foundBracket && c == ' '){

                    // This is the end of the prompt
                    m_simBuf[m_simBufPos] = 0;
                    diagStreamPrintLn();
                    return true;
                }else if(c == '>'){
                    foundBracket = true;
                }
            }else if(c == '\n' && m_simBufPos == 0){
                replyStarted = true;
            }
        }
    }
    diagStreamPrintLn(F("<timeout>"));
    m_simBuf[m_simBufPos] = 0;
    return false;
}

// continually checks, up to the timeout, for successful network
// registration. depending on network conditions, this can take a while.
//
// returns true on success, or false on timeout.
bool GprsSender::waitForNetworkReg(uint32_t timeout) {

    // The reply to CREG is "<CR><LF>+CREG: <n>,<stat><CR><LF>"
    // Where stat is the status which corresponds to the following (from the
    // manual):
    // 0 = Not registered, MT is not currently searching an operator to register
    //     to
    // 1 = Registered, home network
    // 2 = Not registered, but MT is currently trying to attach...
    // 3 = Registration denied
    // 4 = Unknown
    // 5 = Registered, roaming

    // 1 and 5 are considered success for our purposes

    uint32_t timestamp = millis() + timeout;
    int statusCode = -1;
    bool success = false;
    while(!success && !timedOut(timestamp)){
        sendCommand(F("AT+CREG?"));
        delayAndWdtReset(DEFAULT_TIMEOUT_MS);
        if(m_serialStream->find("+CREG: ")){
            if(m_serialStream->find(",")){
                statusCode = m_serialStream->parseInt();
                if(statusCode == 1 || statusCode == 5){
                    success = true;
                }
            }
        }
    }
    if(!success){
        diagStreamPrintLn(F("<--<timeout>"));
    }
    return success;
}

// returns signal strength (RSSI). according to the SIM module information,
// the values essentially go from 0-31 with higher being better. the value
// 99 is reserved for "not known" or "not detectable". reproduced here:
//
// 0:    -115 dBm or less
// 1:    -111 dBm
// 2-30: -110 ... -54 dBm
// 31:   -52 dBm or greater
// 99:   not known or not detectable
int GprsSender::signalStrength( uint32_t timeout ) {

    // The response will be something like: +CSQ: <rssi>,<ber>
    // We're looking for the rssi value
    int signalStrength = -1;
    sendCommand(F("AT+CSQ"));
    m_serialStream->setTimeout(timeout); // Set serial timeout
    if(m_serialStream->find("+CSQ: ")){
        signalStrength = m_serialStream->parseInt();
    }
    m_serialStream->setTimeout(1000); // Set serial timeout back to default

    return signalStrength;
}

// send a command and then wait for the reply
// returns false on timeout
bool GprsSender::sendCommandWaitForReply(const __FlashStringHelper *command,
    const __FlashStringHelper *reply, uint32_t timeout ) {

    sendCommand(command);
    return waitForReply(reply, timeout);
}

// delay and reset the watchdog timer as we go
// this prevents a delay in our code from accidentally
// triggering a watchdog reset
void GprsSender::delayAndWdtReset(uint32_t ms) {

    // The minimum watchdog timeout is 15 ms, so we need to be under
    // this. 10 Seems reasonable
    const unsigned long minDelay = 10;

    // delay in sets of the minimum delay, resetting the watchdog timer in
    // between
    while(ms > minDelay){
        wdt_reset(); // Reset watchdog timer
        delay(minDelay);
        ms -= minDelay;
    }
    delay(ms);
}

// Reads a line formatted <CR><LF>reply<CR><LF> from the SIM module
// returns the number of bytes read or -1 on timeout
int GprsSender::readLine(uint32_t timeout) {
    uint32_t timestamp = millis() + timeout;
    m_simBufPos = 0;
    bool replyStarted = false;
    while(!timedOut(timestamp)){
        while(m_serialStream->available()){
            wdt_reset();
            char c = m_serialStream->read();
            if(c > 31 && c < 128){
                diagStreamPrint(c);
            }else{
                diagStreamPrint(c, HEX);
            }

            // The replies come as <CR><LF>reply<CR><LF>
            if(replyStarted && (m_simBufPos < SIM_BUF_LEN)){
                if(c == '\n'){
                    m_simBuf[m_simBufPos] = 0;
                    return m_simBufPos;
                }else if(c != '\r'){
                    m_simBuf[m_simBufPos] = c;
                    m_simBufPos++;
                }
            }else if(c == '\n' && m_simBufPos == 0){
                replyStarted = true;
            }
        }
    }
    diagStreamPrintLn(F("<timeout>"));
    m_simBuf[m_simBufPos] = 0;
    return -1;
}

// attempts to parse the HTTP status code from the server response
int GprsSender::readStatusCode(uint32_t timeout) {

    // The response will be something like:
    // HTTP/1.1 <Status Code> <Text Description><CR><LF>other stuff
    // We're just going to look for a number that's greater than 100 and less
    // than 600
    int statusCode = -1;
    m_serialStream->setTimeout(timeout); // Set serial timeout
    if(m_serialStream->find("HTTP/1.1 ")){
        statusCode = m_serialStream->parseInt();
    }
    m_serialStream->setTimeout(1000); // Set serial timeout back to default

    if(statusCode < 100 || statusCode > 600){
        return -1;
    }

    return statusCode;
}

#endif // _MANYLABS_GPRS_SENDER_H_
