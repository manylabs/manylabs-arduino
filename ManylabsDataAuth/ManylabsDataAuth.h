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

// convert sha256 hash to hex
void hexString( char *output, uint8_t* input, int inputLength ) {
  PGM_P hexChars = PSTR("0123456789abcdef");
  int j = 0;
  for (int i = 0; i < inputLength; i++) {
    output[ j++ ] = pgm_read_byte( &(hexChars[ input[i] >> 4 ]) );
    output[ j++ ] = pgm_read_byte( &(hexChars[ input[i] & 0xf ]) );
  }
}

void createHexHash(const __FlashStringHelper *privateKey, const char *data, char *output){
  Sha256.init();
  Sha256.print( privateKey );
  Sha256.print( F(";") );
  Sha256.print( data );
  uint8_t *hash = Sha256.result();
  // convert hash to hexadecimal
  hexString( output, hash, 32 );
  output[ 64 ] = 0;
}

// write the header to a stream such as Serial
// This includes the /r/n at the end of the header line
void writeAuthHeader(const __FlashStringHelper *publicKey,
  const __FlashStringHelper *privateKey, const char *data, Stream &stream){

  // Header looks like: manydata-authentication: publicKey:hashedData
  // where hashedData is the sha256 hash of: privateKey;data
  stream.print( F("manydata-authentication: ") );
  stream.print( publicKey );
  stream.print( F(":") );

  char hexHash[ 65 ]; // This _must_ be 65 characters
  createHexHash(privateKey, data, hexHash);

  stream.print(hexHash);
  stream.print(F("\r\n"));
}

// write the header to a supplied buffer
// this includes the /r/n at the end of the header line
//
// Note: The required buffer size is 93 + the length of your public key
//
// returns true if the buffer was large enough. If the buffer was too small,
// it will return false and the header in the buffer will be truncted.
bool writeAuthHeader(const __FlashStringHelper *publicKey,
  const __FlashStringHelper *privateKey, const char *data, char *buffer, int size){

  // Header looks like: manydata-authentication: publicKey:hashedData
  // where hashedData is the sha256 hash of: privateKey;data
  strlcat_P(buffer, PSTR("manydata-authentication: "), size );
  strlcat_P(buffer, (const char*)publicKey, size );
  strlcat_P(buffer, PSTR(":"), size );

  char hexHash[ 65 ]; // This _must_ be 65 characters
  createHexHash(privateKey, data, hexHash);

  strlcat(buffer, hexHash, size);
  size_t resultSize = strlcat_P(buffer, PSTR("\r\n"), size);

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
