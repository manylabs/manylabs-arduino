// Manylabs WireGarden Arduino Library 0.4.0
// some code adapted from outside sources
// other code copyright ManyLabs 2011-2014
// dual license:
// (1) MIT license (without external GPL'd libraries)
// (2) GPL license (with external GPL'd libraries)
#ifndef _WIREGARDEN_ERROR_CODES_H_
#define _WIREGARDEN_ERROR_CODES_H_


// command errors
#define MEC_BAD_CHECKSUM F("error: bad checksum (100)")
#define MEC_INVALID_COMMAND F("error: invalid command (101)")


// general device errors
#define MEC_INVALID_PIN F("error: invalid pin (200)")
#define MEC_INVALID_DEVICE_TYPE F("error: invalid device type (201)")
#define MEC_DEVICE_NOT_FOUND F("error: device not found (202)")
#define MEC_INVALID_VALUES F("error: invalid values (203)")
#define MEC_DEVICE_LIMIT F("error: device limit reached (204)")


// device-specific errors
#define MEC_DS18B20_BAD_CRC F("error: bad crc (1001)")
#define MEC_DS18B20_BAD_DEVICE F("error: bad device (1002)")
#define MEC_TSL2561_NOT_FOUND F("error: not found (1003)")
#define MEC_NO_THERMOCOUPLE F("error: no thermocouple (1004)")


#endif // _WIREGARDEN_ERROR_CODES_H_
