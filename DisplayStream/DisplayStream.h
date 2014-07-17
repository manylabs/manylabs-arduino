// Manylabs DisplayStream Arduino Library 0.4.0
// copyright Manylabs 2014
// MIT license
#ifndef _MANYLABS_DISPLAY_STREAM_H_
#define _MANYLABS_DISPLAY_STREAM_H_
#include <Wire.h>
#include <SeeedOLED.h>


#ifndef STREAM_DISPLAY_ROWS
	#define STREAM_DISPLAY_ROWS 8
#endif
#ifndef STREAM_DISPLAY_COLS
	#define STREAM_DISPLAY_COLS 16
#endif


class DisplayStream : public Stream {
public:

	// initialize the data stream; note: must call init() during setup
	DisplayStream() {
		_row = 0;
		_col = 0;
		for (int i = 0; i < STREAM_DISPLAY_ROWS + 1; i++) {
			_text[ i ][ 0 ] = 0;
		}
	}

	// initialize the OLED display
	void init() {
		Wire.begin();
		SeeedOled.init();
		SeeedOled.clearDisplay();
		SeeedOled.setNormalDisplay();
		SeeedOled.setPageMode();
	}

	// redraw the display using the current text (excludes the current line)
	void draw() {
		SeeedOled.clearDisplay();
		int row = _row; // start with the current row, but increment first, so we're actually starting with the next row after the current
		for (int i = 0; i < STREAM_DISPLAY_ROWS; i++) {
			row++;
			if (row == STREAM_DISPLAY_ROWS + 1)
				row = 0;
			SeeedOled.setTextXY( i, 0 );
			SeeedOled.putString( _text[ row ] );
		}
	}

	// write a character into the text buffer; if it is a newline, moves to next line and redraws the display
	size_t write( byte data ) {
		if (data == 10) {
			_text[ _row ][ _col ] = 0;
			_row++;
			if (_row == STREAM_DISPLAY_ROWS + 1)
				_row = 0;
			_col = 0;
			draw();
		} else if (data >= ' ' && _col < STREAM_DISPLAY_COLS) {
			_text[ _row ][ _col ] = data;
			_col++;
		}
	}

	void flush() {
		draw();
	}

	// not implemented (used for reading)
	int read() { return 0; }
	int available() { return 0; }
	int peek() { return 0; }

private:

	// text buffer
	// we have an extra row for the current row (not display) and an extra column for a zero terminator
	char _text[ STREAM_DISPLAY_ROWS + 1 ][ STREAM_DISPLAY_COLS + 1 ];

	// position of current in text buffer
	int _row;
	int _col;
};


#endif // _MANYLABS_DISPLAY_STREAM_H_
