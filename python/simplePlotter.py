# This is a simple sample script for plotting data from an Arduino.
# It assumes that your Arduino sends comma-separated sensor values.
import datetime
import serial # http://pyserial.sourceforge.net/
from matplotlib import pyplot


# set this to your serial port (e.g. COM3 on windows or /tty.usb/... on other mac/linux)
SERIAL_PORT = "COM3"


# maximum data points to show
MAX_LEN = 100


# plot data on a window
def runPlotting():

    # open the serial port
    serialConnection = serial.Serial( SERIAL_PORT, timeout=2.0 )

    # prep plotting
    pyplot.ion()
    y = [0] * MAX_LEN
    plot, = pyplot.plot( y )
    
    # do until user interrupts
    while True:
        data = serialConnection.readline().strip()
        if data:
            print data
            values = [float( v ) for v in data.split( "," )]
            y = y[ 1: ]
            y.append( values[ 0 ] )
            plot.set_ydata( y )
            pyplot.ylim( [min( y ), max( y )] )
            pyplot.draw()


# if run as top-level script
if __name__ == "__main__":
    try: 
        runPlotting()
    except KeyboardInterrupt:
        pass