// Manylabs DisplayStream Arduino Library 0.4.0
// copyright Manylabs 2014
// MIT license
#ifndef _MANYLABS_DATA_FLASH_STREAM_H_
#define _MANYLABS_DATA_FLASH_STREAM_H_
#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "Sodaq_dataflash.h"


// size of a buffer for holding data read from flash memory
#define READ_BUFFER_SIZE 16


// standard pin definitions for the sodaq data flash IC
#define DF_MOSI          11
#define DF_MISO          12
#define DF_SPICLOCK      13
#define DF_SLAVESELECT   10


// The DataFlashInputStream class provides a stream interface for reading from a flash memory chip.
// It is a wrapper for the Sodaq_dataflash library.
// It is intended for text data (not binary); it uses a 0 byte to represent the end of file.
class DataFlashInputStream : public Stream {
public:

	// prepare to read, starting at beginning of flash memory
	DataFlashInputStream() {
		_position = 0;
		_eof = false;
	}

	// read a byte from the flash
	int read() {
		int bufferPosition = _position % READ_BUFFER_SIZE;
		if (bufferPosition == 0) {
			int pagePosition = _position % DF_PAGE_SIZE;
			if (pagePosition == 0) { // start a new page, need to request from flash
				int pageIndex = _position / DF_PAGE_SIZE;
				if (pageIndex > DF_NR_PAGES)
					return 0;
				dflash.readPageToBuf1( pageIndex );
			}
			dflash.readStrBuf1( pagePosition, _readBuffer, READ_BUFFER_SIZE );
		}
		_position++;
		int v = _readBuffer[ bufferPosition ];
		if (v == 0) {
			_eof = true;
		}
		return v;
	}

	// fix: implement soon
	int available() { return _eof ? 0 : 1; }

	// fix: implement soon
	int peek() { return 0; }

	// not implemented (used for writing)
	void flush() {}
	size_t write( byte data ) { return 0; }

	// reset to starting position
	void reset() {
		_position = 0;
		_eof = false;
	}

private:

	// a small buffer of values retrieved from flash (smaller than a page, common multiple of page size)
	byte _readBuffer[ READ_BUFFER_SIZE ];

	// current position within flash memory
	unsigned long _position;

	// true if at end of file (i.e. encountered a zero character)
	bool _eof;
};


// The DataFlashInputStream class provides a stream interface for writing to a flash memory chip.
// It is a wrapper for the Sodaq_dataflash library.
// It is intended for text data (not binary); it uses a 0 byte to represent the end of file.
class DataFlashOutputStream : public Stream {
public:

	// prepare to write starting at beginning of flash memory
	DataFlashOutputStream() {
		_position = 0;
		_closed = false;
		_checkSum = 0;
	}

	// close the stream by writing a zero (EOF marker) and storing current page
	~DataFlashOutputStream() {
		close();
	}

	// close the stream by writing a zero (EOF marker) and storing current page
	void close() {
		if (_closed == false) {
			write( 0 ); // write an eof marker

			// if we're not at the beginning of a page, we have some data to write out
			int pagePosition = _position % DF_PAGE_SIZE;
			if (pagePosition) {
				int pageIndex = _position / DF_PAGE_SIZE;
				if (pageIndex < DF_NR_PAGES) {
					dflash.writeBuf1ToPage( pageIndex );
				}
			}
			_closed = true;
		}
	}

	// write a byte to flash memory
	size_t write( byte data ) {
		if (_closed) return 0;
		_checkSum += data;
		int pagePosition = _position % DF_PAGE_SIZE;
		dflash.writeByteBuf1( pagePosition, data );
		if (pagePosition + 1 == DF_PAGE_SIZE) {
			int pageIndex = _position / DF_PAGE_SIZE;
			if (pageIndex < DF_NR_PAGES) {
				dflash.writeBuf1ToPage( pageIndex );
			}
		}
		_position++;
		return 1;
	}

	// write multiple bytes to flash memory;
	// not tested; probably better to do buffering on singleton writes and remove this
	void write( byte *data, int length ) {
		if (_closed == false) {
			while (length > 0) {
				int pagePosition = _position % DF_PAGE_SIZE;
				int writeLength = length;
				if (writeLength + pagePosition > DF_PAGE_SIZE) {
					writeLength = DF_PAGE_SIZE - pagePosition;
				}
				dflash.writeStrBuf1( pagePosition, data, writeLength );
				if (pagePosition + writeLength == DF_PAGE_SIZE) {
					int pageIndex = _position / DF_PAGE_SIZE;
					if (pageIndex < DF_NR_PAGES) {
						dflash.writeBuf1ToPage( pageIndex );
					}
				}
				_position += writeLength;
				data += writeLength;
				length -= writeLength;
			}
		}
	}

	// write out a check sum and newline
	void endLine() {
        int checkSum = _checkSum;
		write( '|' );
		print( checkSum );
		print( "\n" );
		_checkSum = 0;
	}

	// store an eof marker (without moving write position ahead) send current buffer to storage
	void flush() {
		int pagePosition = _position % DF_PAGE_SIZE;
		dflash.writeByteBuf1( pagePosition, 0 ); // write an eof marker
		int pageIndex = _position / DF_PAGE_SIZE;
		if (pageIndex < DF_NR_PAGES) {
			dflash.writeBuf1ToPage( pageIndex );
		}
	}

	// seek to end of file
	void seekToEndOfFile() {
		unsigned long position = 0;
		DataFlashInputStream inStream;
		while (inStream.available()) {
			int v = inStream.read(); // this reading will ensure that we have loaded the current page before we start writing
			if (v)
				position++;
		}
		_position = position;
	}

	// erase all data (write an EOF/zero at beginning of flash)
	void erase() {
		_position = 0;
		write( 0 );
		flush();
		_position = 0;
		_checkSum = 0;
	}

	// get checksum value
	inline byte checkSum() const { return _checkSum; }

	// set checksum value to zero
	inline void resetCheckSum() { _checkSum = 0; }

	// get current file position
	inline unsigned long position() const { return _position; }

	// not implemented (used for reading)
	int read() { return 0; }
	int available() { return 0; }
	int peek() { return 0; }

private:

	// current position within flash memory
	unsigned long _position;

	// current checksum of written data
	byte _checkSum;

	// true if file has been closed
	bool _closed;
};


// The DataFlashRetriever class provides a simple interface for retrieving blocks of data from flash memory
// and transmitting them over the serial port.
class DataFlashRetriever {
public:

	DataFlashRetriever() {
		m_lastEndLineIndex = 0;
		m_lineIndex = 0;
	}

	// retrieve the contents of the flash memory and sends it across the serial port
	void retrieve( long startLineIndex, long lineCount ) {

		// check whether we can pick up where we left off
		if (startLineIndex != m_lastEndLineIndex + 1) {
			m_inputStream.reset();
			m_lineIndex = 0;
			m_value = m_inputStream.read();
		}

		// helper variables
		int value = m_value;
		boolean atEndOfLine = false;
		boolean atEndOfFile = false;
		bool firstData = true;
		int endLineIndex = startLineIndex + lineCount - 1;

		// display initial messages
		if (startLineIndex == 0)
			Serial.println( "#beginData#" );
		Serial.println( "#beginBlock#" );

		// loop until end of data or until reach endLineIndex
		while (m_inputStream.available()) {
			if (value == 0) {
				atEndOfFile = true;
				break;
			}
			if (value == 10 || value == 13) {
				atEndOfLine = true;
			} else {
				if (atEndOfLine) {
					m_lineIndex++;
					if (m_lineIndex > endLineIndex) {
						break;
					}
					if (m_lineIndex > startLineIndex) {
						Serial.print( m_lineIndex );
						Serial.print( ":" );
					}
				}
				atEndOfLine = false;
			}

			// if character is printable and we're past startLineIndex, display the character
			if ((value == 10 || value == 13 || (value > 32 && value < 128)) && m_lineIndex >= startLineIndex) {
				if (firstData) {
					Serial.print( startLineIndex );
					Serial.print( ":" );
					firstData = false;
				}
				Serial.write( value );
			}
			value = m_inputStream.read();
		}
		Serial.println( "#endBlock#" );
		if (atEndOfFile || m_inputStream.available() == false) {
			Serial.println( "#endData#" );
		}
		m_lastEndLineIndex = endLineIndex;
		m_value = value; // store this for next time around; alternatively, could handle end-of-line on 10 or 13 and not always read past it
	}

private:

	// private data
	DataFlashInputStream m_inputStream;
	long m_lineIndex;
	long m_lastEndLineIndex;
	int m_value;
};


// temporarily re-add old version of code until sketches are migrated
void retrieveStorage( int startLineIndex, int lineCount ) {
	int lineIndex = 0;
	boolean atEndOfLine = true;
	Serial.println( "#beginData#" );
	DataFlashInputStream inStream;
	while (inStream.available()) {
		int value = inStream.read();
		if (value == 0)
			break;
		if (value == 10 || value == 13) {
			atEndOfLine = true;
		} else {
			if (atEndOfLine) {
				Serial.print( lineIndex );
				Serial.print( ":" );
				lineIndex++;
			}
			atEndOfLine = false;
		}
		if (value == 10 || value == 13 || (value > 32 && value < 128))
			Serial.write( value );
	}
    Serial.println( "#endData#" );
}


#endif // _MANYLABS_DATA_FLASH_STREAM_H_
