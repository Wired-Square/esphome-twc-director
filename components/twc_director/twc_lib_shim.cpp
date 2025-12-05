// components/twc_director/twc_lib_shim.cpp
//
// Shim to make sure the pure-C TWC/HPWC library in lib/twc/*.c
// is compiled and linked into the ESPHome firmware.
//
// We compile the C files as C++ but give them C linkage with
// extern "C" so the symbols match the headers.

extern "C" {
#include "twc/twc_core.c"
#include "twc/twc_device.c"
#include "twc/twc_frame.c"
#include "twc/twc_protocol.c"
}
