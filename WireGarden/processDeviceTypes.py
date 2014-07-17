import json
import distutils.dir_util


# a class for holding information about a device type
class DeviceType( object ):
    def __init__( self ):
        self.number = 0
        self.name = ''
        self.define = ''
        self.description = ''
        self.subTypes = []
        self.pinType = ''
        self.notes = ''
        self.units = []
        self.docUrl = ''
        self.optionDefine = ''


# load device types from a name-value text file
def loadDeviceTypes( fileName ):
    inputFile = open( fileName )
    deviceType = None
    deviceTypes = []
    for line in inputFile:
        if (line[0] is not "#") and (':' in line):
            parts = line.split( ':', 1 )
            name = parts[ 0 ].strip()
            value = parts[ 1 ].strip()
            if name and value:
                if name == "number":
                    deviceType = DeviceType()
                    deviceTypes.append( deviceType )
                    deviceType.number = int( value )
                elif name == "name":
                    deviceType.name = value
                elif name == "define":
                    deviceType.define = value
                elif name == "optionDefine":
                    deviceType.optionDefine = value
                elif name == "description":
                    deviceType.description = value
                elif name == "pinType":
                    deviceType.pinType = value
                elif name == "subTypes":
                    deviceType.subTypes = [v.strip() for v in value.split( "," )]
                elif name == "notes":
                    deviceType.notes = value
                elif name == "units":
                    deviceType.units = [v.strip() for v in value.split( "," )]
                elif name == "docUrl":
                    deviceType.docUrl = value
    return deviceTypes


# create a C/C++ header file with #defines for device types
def createHeader( deviceTypes, outputFileName ):
    writtenDefines = []
    outputFile = open( outputFileName, "wb" ) # binary mode so we get unix line endings
    outputFile.write( "// this file is generated automatically from deviceTypes.txt\n" )
    outputFile.write( "#ifndef _WIREGARDEN_DEVICE_TYPES_H_\n" )
    outputFile.write( "#define _WIREGARDEN_DEVICE_TYPES_H_\n\n\n" )
    for deviceType in deviceTypes:
        # Don't include duplicate defines in the header. Some devices like knob and
        # analog input share the same define.
        if deviceType.define not in writtenDefines:
            outputFile.write( "#define %s %d\n" % (deviceType.define, deviceType.number) )
            writtenDefines.append( deviceType.define )
    outputFile.write( "\n\n#endif // _WIREGARDEN_DEVICE_TYPES_H_\n" )


# create a JS object containing device type information
def createJavaScript( deviceTypes, outputFileName ):
    outputFile = open( outputFileName, "wb" ) # binary mode so we get unix line endings
    outputFile.write( "// this file is generated automatically from deviceTypes.txt\n" )
    outputFile.write( "_hardwareDeviceTypes = [\n" )
    first = True
    for deviceType in deviceTypes:
        
        if not first:
            outputFile.write( ",\n" )
        else:
            first = False

        deviceTypeInfo = {
            'number': deviceType.number, 
            'name': deviceType.name,
            'description': deviceType.description,
            'subTypes': deviceType.subTypes,
            'pinType': deviceType.pinType,
            'units': deviceType.units,
            'docUrl': deviceType.docUrl,
            'optionDefine': deviceType.optionDefine
        }
        outputFile.write( "%s" % json.dumps( deviceTypeInfo ) )
    outputFile.write( "\n];\n" )


# if run as top-level script
if __name__ == '__main__':
    deviceTypes = loadDeviceTypes( "deviceTypes.txt" )
    createHeader( deviceTypes, "library/wgDeviceTypes.h" )
    createJavaScript( deviceTypes, "hardwareDeviceTypes.js" )

