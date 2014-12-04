#include "ManylabsDataAuth.h"
#include "sha256.h"

#define PUBLIC_KEY  "123456789"
#define PRIVATE_KEY "987654321"

#define LARGE_BUFFER_SIZE 102
char largeBuffer[ LARGE_BUFFER_SIZE ];

#define SMALL_BUFFER_SIZE 50
char smallBuffer[ SMALL_BUFFER_SIZE ];

char data[] = "dataSet=1&temperature=21&humidity=50";

void setup() {
    Serial.begin(9600);
    Serial.println("Starting Tests");
    Serial.println("==============");

    // Write the header directly to the provided Stream.
    Serial.println("Write header to stream:");
    writeAuthHeader(F(PUBLIC_KEY), F(PRIVATE_KEY), data, Serial);
    Serial.println("==============");

    // Write the header to a buffer that's too small
    Serial.println("Write header to small buffer:");
    bool bufferLargeEnough = writeAuthHeader(F(PUBLIC_KEY), F(PRIVATE_KEY),
        data, smallBuffer, SMALL_BUFFER_SIZE);
    Serial.println(smallBuffer);
    if(!bufferLargeEnough){
        Serial.println("Buffer is too small.");
    }
    Serial.println("==============");

    // Write the header to a large buffer
    Serial.println("Write header to large buffer:");
    bufferLargeEnough = writeAuthHeader(F(PUBLIC_KEY), F(PRIVATE_KEY), data,
        largeBuffer, LARGE_BUFFER_SIZE);
    Serial.print(largeBuffer);
    if(!bufferLargeEnough){
        Serial.println("Buffer is too small.");
    }
    Serial.println("==============");
    Serial.println("Tests Complete");
}

void loop() {
}
