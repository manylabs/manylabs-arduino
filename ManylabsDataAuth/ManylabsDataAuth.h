// Manylabs Data API Authentication Library 0.4.0
// copyright Manylabs 2014; MIT license
// --------
// This library provides a simple way to generate authenication headers for the
// Manylabs Data API
#ifndef _MANYLABS_DATA_AUTH_H_
#define _MANYLABS_DATA_AUTH_H_
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include <sha256.h>
#include <avr/pgmspace.h>

//============================================
// MANYLABS DATA AUTH CLASS DEFINITION
//============================================

class ManylabsDataAuth : public Print{
public:

  ManylabsDataAuth();

  // Initialize the ManylabsDataAuth object with the given keys.
  void init( const __FlashStringHelper *publicKey,
    const __FlashStringHelper *privateKey );

  // Reset the ManylabsDataAuth object. This should be called every time you
  // send data, before printing new data to the ManylabsDataAuth
  void reset();

  // Write data to the ManylabsDataAuth. You can use all the same print methods
  // you can with the Serial object.
  virtual size_t write( uint8_t );
  using Print::write;

  // Write the auth header to the given stream. This includes "\r\n" at the end
  // of the line.
  void writeAuthHeader( Stream &stream );

  // Write the auth header to the given buffer. Specify the size of the buffer
  // in size. This includes "\r\n" at the end of the line.
  //
  // Note: The required buffer size is 93 + the length of your public key
  //
  // returns true if the buffer was large enough. If the buffer was too small,
  // it will return false and the header in the buffer will be truncted.
  bool writeAuthHeader( char *buffer, int size );

private:

  // get the resulting sha256 hash and convert it to hexidecimal
  void createHexHash( char *output );

  // convert sha256 hash to hex
  void hexString( char *output, uint8_t* input, int inputLength );

  // pointers to the keys in Flash memory
  const __FlashStringHelper *m_publicKey;
  const __FlashStringHelper *m_privateKey;
};

//============================================
// MANYLABS DATA AUTH IMPLEMENTATION
//============================================

ManylabsDataAuth::ManylabsDataAuth() {}

// Initialize the ManylabsDataAuth object with the given keys.
void ManylabsDataAuth::init( const __FlashStringHelper *publicKey,
  const __FlashStringHelper *privateKey ){

  m_publicKey = publicKey;
  m_privateKey = privateKey;

  reset();
}

// Reset the ManylabsDataAuth object. This should be called every time you send
// data, before printing new data to the ManylabsDataAuth
void ManylabsDataAuth::reset() {
  Sha256.init();
  Sha256.print( m_privateKey );
  Sha256.print( F(";") );
}

// Write data to the ManylabsDataAuth. You can use all the same print methods
// you can with the Serial object.
size_t ManylabsDataAuth::write( uint8_t data ) {
  Sha256.write(data);
}

// get the resulting sha256 hash and convert it to hexidecimal
void ManylabsDataAuth::createHexHash( char *output ) {
  uint8_t *hash = Sha256.result();
  // convert hash to hexadecimal
  hexString( output, hash, 32 );
  output[ 64 ] = 0;
}

// convert sha256 hash to hex
void ManylabsDataAuth::hexString( char *output, uint8_t* input,
  int inputLength ) {

  PGM_P hexChars = PSTR("0123456789abcdef");
  int j = 0;
  for (int i = 0; i < inputLength; i++) {
    output[ j++ ] = pgm_read_byte( &(hexChars[ input[i] >> 4 ]) );
    output[ j++ ] = pgm_read_byte( &(hexChars[ input[i] & 0xf ]) );
  }
}

// Write the auth header to the given buffer. Specify the size of the buffer
// in size.
void ManylabsDataAuth::writeAuthHeader( Stream &stream ) {

  // Header looks like: manydata-authentication: publicKey:hashedData
  // where hashedData is the sha256 hash of: privateKey;data
  stream.print( F("manydata-authentication: ") );
  stream.print( m_publicKey );
  stream.print( F(":") );

  char hexHash[ 65 ]; // This _must_ be 65 characters
  createHexHash( hexHash );

  stream.print( hexHash );
  stream.print( F("\r\n") );
}

// Write the auth header to the given buffer. This includes "\r\n" at the end
// of the line. Specify the size of the buffer in size.
//
// Note: The required buffer size is 93 + the length of your public key
//
// returns true if the buffer was large enough. If the buffer was too small,
// it will return false and the header in the buffer will be truncted.
bool ManylabsDataAuth::writeAuthHeader( char *buffer, int size ) {

  // Header looks like: manydata-authentication: publicKey:hashedData
  // where hashedData is the sha256 hash of: privateKey;data
  strlcat_P( buffer, PSTR("manydata-authentication: "), size );
  strlcat_P( buffer, (const char*)m_publicKey, size );
  strlcat_P( buffer, PSTR(":"), size );

  char hexHash[ 65 ]; // This _must_ be 65 characters
  createHexHash( hexHash );

  strlcat( buffer, hexHash, size );
  size_t resultSize = strlcat_P( buffer, PSTR("\r\n"), size );

  // Here we check if the buffer was to small to hold all the data we tried to
  // fill it with. If the resultSize is >= the buffer size, then the data was
  // truncated and the buffer needs to be larger
  if(resultSize >= size){
    return false;
  }
  // Otherwise, the buffer has enough space.
  return true;
}

#endif _MANYLABS_DATA_AUTH_H_
