// Manylabs GprsSender example
// copyright Manylabs 2014; MIT license
// --------
// This example shows how to send data to a data set on Manylabs.org using a
// GPRS/SIM module. We're using the Adafruit Fona, but much of this code would
// be the same for other modules.

// If we were using API keys we'd need to uncomment the next three lines
// #define USE_MANYLABS_DATA_AUTH
// #include <sha256.h>
// #include "ManylabsDataAuth.h"

#include <SoftwareSerial.h>
#include "GprsSender.h"


// Manylabs Data API Keys - If we were using a data set that wasn't open, we'd
// need to use API keys instead. If you share this sketch with someone else,
// don't include your keys.
// #define PUBLIC_KEY  "123456"
// #define PRIVATE_KEY "987654"

// Example data set https://www.manylabs.org/tool/dataLab/?dataSet=540
#define DATA_SET_ID 540

/**
 *  GPRS Sender
 */

// Digital pin 6 is TX from the GPRS module
// Digital pin 7 is RX from the GPRS module
SoftwareSerial gprsSerial(6, 7);
#define GPRS_RESET_PIN  8 // Reset pin connected to digital pin 8
GprsSender gprsSender(GPRS_RESET_PIN, gprsSerial, Serial );

// Parameter buffer to store parameters we'll post to the data set
#define PARAM_BUFFER_LENGTH 100
char paramBuffer[ PARAM_BUFFER_LENGTH ];

// Number of times we've failed to send because of a SIM module error
uint8_t gprsFailCount = 0;

/**
 * Display
 */
// If you have an OLED display, you can uncomment the next line to print info
// there. Otherwise, we'll just use Serial
// #define USE_DISPLAY
#ifdef USE_DISPLAY
  #include <Wire.h>
  #include <SeeedOLED.h>
  #include <DisplayStream.h>
  DisplayStream display;
  #define DISPLAY(x) display.println(x)
#else
  #define DISPLAY(x) Serial.println(x)
#endif

/**
 * Devices
 */

// If you had other sensors, you could add them here.

// This is how long we wait between sending sensor values
#define SENSOR_INTERVAL_MSECS 300000 // 5 * 60 * 1000
unsigned long lastSensorTime = 0; // And the last time we sent them

void setup() {

  Serial.begin(9600);
  gprsSerial.begin(19200);

  delay(1000);

  if(!gprsSender.init(paramBuffer, PARAM_BUFFER_LENGTH, F("truphone.com"))){
    DISPLAY(F("Init Failed"));
  }

  #ifdef USE_DISPLAY
    display.init();
  #endif
    DISPLAY(F("Setup Complete"));

  // Process the sensors once on startup so we don't have to wait a long time
  // to make sure it's working
  delay(1000);
  processSensors();
}

void loop() {
  unsigned long time = millis();

  // If we've waited long enough, process and send the sensors again
  if (time - lastSensorTime > SENSOR_INTERVAL_MSECS) {
    processSensors();
    lastSensorTime = time;
  }
}

void processSensors(){
  DISPLAY(F("Reading Sensors"));

  // Normally you'd read your sensors here. For this example we're just using
  // a random number.
  float randomNumber = random(1001); // Random number between 0 and 1000
  DISPLAY( F("Random Number:") );
  DISPLAY(randomNumber);

  // Add the values to be sent. We have to include the dataSetId and the
  // Random Number. Since we're not computing a timestamp, we'll ask the server
  // to add one for us.
  gprsSender.add("dataSetId", DATA_SET_ID);
  gprsSender.add("addTimestamp", 1);
  gprsSender.add("Random Number", randomNumber);

  DISPLAY( F("Sending") );

  // Send the data. If we were using API keys, we'd call this instead:
  // gprsSender.sendWithManylabsDataAuth(F(PUBLIC_KEY),F(PRIVATE_KEY))
  if(gprsSender.send()){

    // Get the HTTP status code from the server. 201 is a success here because
    // we're creating a new record.
    // http://en.wikipedia.org/wiki/List_of_HTTP_status_codes
    int statusCode = gprsSender.lastStatusCode();
    if(statusCode == 201){
      DISPLAY( F("Complete") );
    }else{
      DISPLAY( F("Status Code: ") );
      DISPLAY(statusCode);
    }
  }else{

    // If we had a problem, check for the reason. If it's 1, something went
    // wrong with the SIM module. If it's 2, something is wrong with the
    // network connection or the server.
    int errorCause = gprsSender.lastErrorCode();
    if(errorCause == 2){
      DISPLAY( F("Server / Network Failure") );
    }else{
      DISPLAY( F("GPRS Failure") );

      // If the SIM module fails several times, it might be a good idea to
      // reboot.
      gprsFailCount++;
      if(gprsFailCount > 2){
        gprsFailCount = 0;
        rebootAndReconnect();
      }
    }
  }
}

void rebootAndReconnect(){

  // Reboot the SIM module
  DISPLAY( F("Rebooting") );
  gprsSender.reboot();

  // Wait for it to reconnect to the cell network
  DISPLAY( F("Reconnecting") );
  if(gprsSender.waitForNetworkReg()){
    DISPLAY( F("Success") );
  }else{
    DISPLAY( F("Failure") );
  }
}
