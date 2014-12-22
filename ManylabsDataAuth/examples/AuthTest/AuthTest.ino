#include "ManylabsDataAuth.h"
#include "sha256.h"

#define PUBLIC_KEY  "123456789"
#define PRIVATE_KEY "987654321"

#define LARGE_BUFFER_SIZE 102
char largeBuffer[ LARGE_BUFFER_SIZE ];

#define SMALL_BUFFER_SIZE 50
char smallBuffer[ SMALL_BUFFER_SIZE ];

ManylabsDataAuth dataAuth;

void setup() {

    Serial.begin(9600);
    Serial.println("Starting Tests");
    Serial.println("==============");

    // Write the header directly to the provided Stream.
    generateAuthHeader();
    Serial.println("Write header to stream:");
    dataAuth.writeAuthHeader(Serial);
    Serial.println("==============");

    // Write the header to a buffer that's too small
    generateAuthHeader();
    Serial.println("Write header to small buffer:");
    bool bufferLargeEnough = dataAuth.writeAuthHeader(smallBuffer, SMALL_BUFFER_SIZE);
    Serial.println(smallBuffer);
    if(!bufferLargeEnough){
        Serial.println("Buffer is too small.");
    }
    Serial.println("==============");

    // Write the header to a large buffer
    generateAuthHeader();
    Serial.println("Write header to large buffer:");
    bufferLargeEnough = dataAuth.writeAuthHeader(largeBuffer, LARGE_BUFFER_SIZE);
    Serial.print(largeBuffer);
    if(!bufferLargeEnough){
        Serial.println("Buffer is too small.");
    }
    Serial.println("==============");
    Serial.println("Tests Complete");
}

void loop() {
}

// Generate the authentication header. This process needs to be completed every
// time you want to write the authentication header.
void generateAuthHeader() {

    // Initialize (or reinitialize) the authenticator.
    dataAuth.init(F(PUBLIC_KEY), F(PRIVATE_KEY));

    // Add data to dataAuth. This works just as you would print data to Serial
    dataAuth.print(F("dataSet=1"));
    dataAuth.print(F("&temperature="));
    dataAuth.print(22.01);
    dataAuth.print(F("&humidity="));
    dataAuth.print(55.3);
}
