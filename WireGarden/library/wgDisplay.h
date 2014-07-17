// Manylabs WireGarden Arduino Library 0.4.0
// some code adapted from outside sources
// other code copyright ManyLabs 2011-2014
// dual license:
// (1) MIT license (without external GPL'd libraries)
// (2) GPL license (with external GPL'd libraries)
#ifndef _WIREGARDEN_DISPLAY_H_
#define _WIREGARDEN_DISPLAY_H_
#include <Wire.h>
#include <SeeedOLED.h>


// display parameters
#define MAX_DISPLAY_ROWS 8
#define MAX_LABEL_LENGTH 8


// initialize display module (displays values on OLED or similar device)
void initDisplay() {
	Wire.begin();
	SeeedOled.init();
	SeeedOled.clearDisplay();
	SeeedOled.setNormalDisplay();
	SeeedOled.setPageMode();
}


// clear the screen
void clearDisplay() {
	SeeedOled.clearDisplay();
}


// set a row of the display with the given label and value to the given decimalPlaces; row is a zero based index
void displayRow( int row, char *label, float value, int decimalPlaces ){
	if(row < MAX_DISPLAY_ROWS){
		char maxLabel[ MAX_LABEL_LENGTH + 1 ];
		strncpy( maxLabel, label, MAX_LABEL_LENGTH ); // copy at most MAX_LABEL_LENGTH characters (padding remainder with zeros)
		maxLabel[ MAX_LABEL_LENGTH ] = 0; // add zero terminator in case label was longer than MAX_LABEL_LENGTH
		SeeedOled.setTextXY( row, 0 );
		SeeedOled.putString( maxLabel );
		SeeedOled.setTextXY( row, strlen( maxLabel ) );
		SeeedOled.putString( ":" );
		SeeedOled.setTextXY( row, MAX_LABEL_LENGTH + 1 );
		SeeedOled.putString( "       " );
		SeeedOled.setTextXY( row, MAX_LABEL_LENGTH + 1 );
		SeeedOled.putFloat( value, decimalPlaces );
	}
}


#endif // _WIREGARDEN_DISPLAY_H_
