// Manylabs WireGarden Arduino Library 0.4.0
// some code adapted from outside sources
// other code copyright Manylabs 2011-2014
// dual license:
// (1) MIT license (without external GPL'd libraries)
// (2) GPL license (with external GPL'd libraries)

// This library provides a uniform interface for a variety of sensors and other
// hardware. It includes a command processing module and other communication tools.
// It depends on a variety of third-party libraries (for different hardware devices).
// This library intended to be a general purpose system used by a variety of Manylabs
// sketches.

// the library is implemented in the following header files
// (we use header file implementations in order to allow better customization from within a sketch)
#include "library/wgErrorCodes.h"
#include "library/wgDeviceTypes.h"
#include "library/wgSend.h"
#include "library/wgCommands.h"
#include "library/wgDevices.h"
#include "library/wgDeviceCommands.h"
#ifdef USE_DISPLAY
#include "library/wgDisplay.h"
#endif
