manylabs-arduino
================

This is a collection of libraries for the Arduino. To use these libraries copy 
them into the libraries folder in your Arduino IDE directory. 

Some of the code is based on code written by others. We have tried to provide 
references for uses of outside code; let us know if you see anything that needs
additional references.

If you have questions about the code, email <support@manylabs.org>.

## WireGarden

This library provides a uniform interface for a variety of sensors and other
hardware. It includes a command processing module and other communication tools.
It depends on a variety of third-party libraries (for different hardware devices).
This library intended to be a general purpose system used by a variety of Manylabs
sketches.

## WifiSender

This library provides a simple interface for sending data to a server via WiFi.
It depends on the WiFly library from Seeed Studio.

## DataFlashStream

This library allows file-like input/output using the flash memory device found
on the SODAQ board. It depends on the dataflash library from the SODAQ developers.

## DisplayStream

This library allows you to use an OLED display as a serial-like stream object for 
displaying data. It is intended for diagnostic purposes in cases where a serial
connection isn't useable. It depends on the Seeed OLED library.

## BLEShieldStream

This library provides a serial-like interface to the BLE Shield from Red Bear Lab.
It depends on the BLE library provided by Red Bear Lab.