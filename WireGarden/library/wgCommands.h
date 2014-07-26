// Manylabs WireGarden Arduino Library
// some code adapted from outside sources
// other code copyright ManyLabs 2011-2014
// dual license:
// (1) MIT license (without external GPL'd libraries)
// (2) GPL license (with external GPL'd libraries)
#ifndef _WIREGARDEN_COMMANDS_H_
#define _WIREGARDEN_COMMANDS_H_
#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "wgSend.h"
#include "wgErrorCodes.h"


// command module parameters
#define INPUT_BUFFER_LENGTH 41
#define MAX_COMMAND_ARGS 10


//============================================
// STRING UTILITIES
//============================================


// get index of given character; return -1 if not found
// fix(smaller): replace with strchr
inline int indexOf( const char *str, char c ) {
	int index = 0;
	while (true) {
		char s = *str++;
		if (s == c)
			return index;
		if (s == 0)
			break;
		index++;
	}
	return -1;
}


// a simple helper macro for string comparision
#define strEq( a, b ) (strcmp( a, b ) == 0)


//============================================
// COMMAND PROCESSING
//============================================


// serial input buffer
char g_inputBuffer[ INPUT_BUFFER_LENGTH ];
int g_inputIndex = 0;
boolean g_enableAcks = false;


// the command execution callback
boolean (*g_executeCommand)( const char *command, byte argCount, char *args[] ) = NULL;


// display amount of free memory
// updated from: http://jeelabs.org/2011/05/22/atmega-memory-use/
void checkMem() {
	extern int __heap_start, *__brkval;
	Serial.print( "i:" );
	Serial.println( (int)(SP)  - (__brkval == 0 ? (int) &__heap_start : (int) __brkval)); // display free memory
}


// check for general commands (commands that are not handled in other modules)
void generalCommand( const char *command, byte argCount, char *args[] ) {

	// test command
	if (strEq( command, "tc" ) && argCount >= 1) {
		sendString( "tr:" ); // test response
		for (int i = 0; i < argCount; i++) {
			if (i)
				sendString( "," );
			sendString( args[ i ] );
		}
		sendString( "\n" );

	// library version and board type
	} else if (strEq( command, "ver" ) && argCount == 0) {
		sendString( "i:" );
#ifdef SKETCH_NAME
		sendString( SKETCH_NAME );
#endif
		sendString( ",0.4.0," );
#if defined(WG_BOARD_NAME)
		sendString(WG_BOARD_NAME);
		sendString("\n");
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega640__)
		sendString( "mega2560\n" ); // mega or compatible
#elif defined(__AVR_ATmega328P__)
		sendString( "uno\n" ); // uno or compatible
#endif

	// memory check
	} else if (strEq( command, "m" ) && argCount == 0) {
		checkMem();

	// enable acks
	} else if (strEq( command, "ea" ) && argCount == 0) {
		g_enableAcks = true;

	// disable acks
	} else if (strEq( command, "da" ) && argCount == 0) {
		g_enableAcks = false;
	} else {
		sendError( MEC_INVALID_COMMAND );
	}
}


// parse the incoming serial buffer and execute any command it contains
void processMessage() {
	g_inputBuffer[ g_inputIndex ] = 0; // truncate the string
	g_inputIndex = 0;
	char *command = g_inputBuffer;
	char *args[ MAX_COMMAND_ARGS ];
	int argCount = 0;

	// check for checksum
	int pipePos = indexOf( g_inputBuffer, '|' ); // fix(faster): could start at end
	if (pipePos >= 0) {
		byte givenCheckSum = atoi( g_inputBuffer + pipePos + 1 );
		byte computedCheckSum = 0;
		for (int i = 0; i < pipePos; i++)
			computedCheckSum += g_inputBuffer[ i ];
		if (givenCheckSum != computedCheckSum) {
			sendError( MEC_BAD_CHECKSUM );
			return;
		}
	}

	// if acks enabled, send back message
	if (g_enableAcks) {
		sendString( "a:" );
		sendString( g_inputBuffer );
		sendNewLine();
	}

	// remove checksum
	if (pipePos >= 0) {
		g_inputBuffer[ pipePos ] = 0; // remove checksum
	}

	// check for command arguments
	int colonPos = indexOf( g_inputBuffer, ':' );
	if (colonPos > 0) {
		g_inputBuffer[ colonPos ] = 0;
		char *argStr = g_inputBuffer + colonPos + 1;
		int commaPos = 0;
		argCount = 0;
		do {

			// strip leading spaces
			while (*argStr == ' ')
				argStr++;

			// store in argument array
			args[ argCount ] = argStr;

			// find end of arg
			commaPos = indexOf( argStr, ',' );
			char *argEnd = 0;
			if (commaPos > 0) {
				argStr[ commaPos ] = 0;
				argStr += commaPos + 1;
				argEnd = argStr - 2;
			} else {
				argEnd = argStr + strlen( argStr ) - 1;
			}

			// strip off tailing spaces (note: we're protected by 0 that replaced colon)
			while (*argEnd == ' ') {
				*argEnd = 0;
				argEnd--;
			}

			// done with this arg
			argCount++;
		} while (commaPos > 0 && argCount < MAX_COMMAND_ARGS);
	}

	// execute the command now that we have parsed it
	if (command[ 0 ]) {
		if (g_executeCommand) {
			boolean handled = g_executeCommand( command, argCount, args );

			// if not handled in callback, check for other commands
			if (handled == false) {
				generalCommand( command, argCount, args );
			}
		}
	}
}


// process a single incoming command byte; store in buffer or execute command if complete
void feedCommandByte( char c ) {
	if (c == 10 || c == 13) {
		if (g_inputIndex)
			processMessage();
	} else if (g_inputIndex >= INPUT_BUFFER_LENGTH - 1) { // want to be able to write 0 into last position after increment
		g_inputIndex = 0;
	} else {
		g_inputBuffer[ g_inputIndex ] = c;
		g_inputIndex++;
	}
}


// read incoming serial commands; will execute any received commands
void checkSerialCommands() {
	while (Serial.available()) {
		char c = Serial.read();
		if (c > 0) {
			feedCommandByte( c );
		}
	}
}


// initialize the command module (this module)
void initCommands( boolean (*executeCommand)( const char *command, byte argCount, char *args[] ) ) {
	g_executeCommand = executeCommand;
	g_inputBuffer[ 0 ] = 0;
}


#endif // _WIREGARDEN_COMMANDS_H_
