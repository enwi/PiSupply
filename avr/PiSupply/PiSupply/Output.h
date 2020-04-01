#ifndef _OUTPUT_H
#define _OUTPUT_H
#include <common.h>

// Enable serial output
#define SERIAL_OUTPUT

#ifdef SERIAL_OUTPUT
#define if_out_enabled(x) x
#define out(...) print(__VA_ARGS__)
#define outln(...) println(__VA_ARGS__)
#define outf(str) print_P(PSTR(str))
#define outlnf(str) println_P(PSTR(str))
#else
#define if_out_enabled(x)
#define out(...)
#define outln(...)
#define outf(str)
#define outlnf(str)
#endif

#endif
