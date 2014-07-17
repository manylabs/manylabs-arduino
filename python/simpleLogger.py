# This is a simple sample script for logging data from an Arduino.
# It assumes that your Arduino sends comma-separated sensor values.
# (It will blindly log all serial messages, but you'll only have a valid 
# .csv file if the messages are comma-separated or a single number per line.)
import serial # http://pyserial.sourceforge.net/


# set this to your serial port (e.g. COM3 on windows or /tty.usb/... on mac/linux)
SERIAL_PORT = "COM3"


# log data from serial port into a text file
def logSensors():
    
    # open the serial port
    serialConnection = serial.Serial( SERIAL_PORT, timeout=2.0 )
    
    # open the output file (for appending)
    outputFile = open( "sensorLog.csv", "a" )
    
    # loop until user breaks
    while True:
        data = serialConnection.readline().strip()
        if data:
            print data
            outputFile.write( data + "\n" )


# if run as top-level script
if __name__ == "__main__":
    try:
        logSensors()
    except KeyboardInterrupt:
        pass