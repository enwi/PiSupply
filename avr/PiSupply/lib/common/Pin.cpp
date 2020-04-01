/*
 * Pin.cpp
 *
 * Created: 28.07.2017 11:11:48
 *  Author: Jan
 */

#include "Pin.h"
const uint8_t detail::pin_bitmask[] PROGMEM = {
    0x01, // pin  0
    0x02, // pin  1
    0x04, // pin  2
    0x08, // pin  3
    0x10, // pin  4
    0x20, // pin  5
};

const uint8_t detail::pin_timer[] PROGMEM = {
    0, /* 0 - port D */
    0,
    0,
    6, // TIMER2B
    0,
    2, // TIMER0B
    1, // TIMER0A
    0,
    0, /* 8 - port B */
    3, // TIMER1A
    4, // TIMER1B
    5, // TIMER2A
    0,
    0,
    0,
    0, /* 14 - port C */
    0,
    0,
    0,
    0,
};