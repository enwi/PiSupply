/*
 * common.h
 *
 * Created: 18.08.2016 17:13:26
 * Author: Moritz Wirger, Jan Rogall
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#include <assert.h>
#include <avr/wdt.h>

#include "fCpu.h"

// If not enabled, remove all print functions
#define ENABLE_PRINT

//! Prescaler values for sleep times
enum sleep_prescaler : uint8_t
{
    sleep_16ms = 0x00,
    sleep_32ms = 0x01,
    sleep_64ms = 0x02,
    sleep_125ms = 0x03,
    sleep_250ms = 0x04,
    sleep_500ms = 0x05,
    sleep_1s = 0x06,
    sleep_2s = 0x07,
    sleep_4s = 0x20,
    sleep_8s = 0x21
};

constexpr unsigned long long BAUDRATE = 57600ULL; //!< constant that determines the USART0 baudrate

// Calculate BAUD prescaler from baudrate and F_CPU                         // works great until 9600        // works
// great for 57600
constexpr unsigned long long BAUD_PRESCALER = BAUDRATE <= 9600ULL
    ? ((F_CPU / (BAUDRATE * 16UL))) - 1
    : (F_CPU / 8 / BAUDRATE - 1) / 2; //!< constant that determines the USART0 baudrate prescaler value

inline void soft_reset()
{
    wdt_enable(WDTO_15MS);
    while (true)
    {
    }
}

// Function Prototype for wdt_init (place it in init3 section, before main executes to prevent perma-resetting)
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

#include <avr/io.h>
#include <avr/pgmspace.h> // needed for PROGMEM
#include <stdio.h>
#include <stdlib.h>

#include "Pin.hpp"

constexpr MicroPin::StaticDigitalPin<2> statusLED;
extern volatile uint16_t WDT_INTS; //!< Variable that tells how many watchdog interrupts have happened (gets increased
                                   //!< inside the watchdog interrupt vector)

//! Function that attaches the given function to an interrupt and calls it upon that.
//! It does this by setting \ref interrupt0func to the given function
//! \param func Pointer to the function that should be called upon interrupt
void attachInterrupt0(void (*func)());

//! Function that detaches the function set by \ref attachInterrupt0 from an interrupt.
//! So basically speaking it clears the interrupt flag and sets \ref interrupt0func to null
void detachInterrupt0();

//! Function that initializes the WDT with a specified timeout
//! \param prescale Used to specify the bits to set the timeout. Standard timeout is 8s
//! Possible values for prescale
//! |value | timeout |
//! |------|---------|
//! |0x00  | 16ms    |
//! |0x01  | 32ms    |
//! |0x02  | 64ms    |
//! |0x03  | 0.125s  |
//! |0x04  | 0.25s   |
//! |0x05  | 0.5s    |
//! |0x06  | 1.0s    |
//! |0x07  | 2.0s    |
//! |0x20  | 4.0s    |
//! |0x21  | 8.0s    |
//! Note no other values than the ones specified should be used!
void initWDT(sleep_prescaler prescale = sleep_prescaler::sleep_8s);

//! Function that prepares the avr for sleep, let's him sleep and wakes him up
//! \param prescale Used to specify the bits to set the timeout. Standard timeout is 8s
//! Possible values for prescale
//! |value | timeout |
//! |------|---------|
//! |0x00  | 16ms    |
//! |0x01  | 32ms    |
//! |0x02  | 64ms    |
//! |0x03  | 0.125s  |
//! |0x04  | 0.25s   |
//! |0x05  | 0.5s    |
//! |0x06  | 1.0s    |
//! |0x07  | 2.0s    |
//! |0x20  | 4.0s    |
//! |0x21  | 8.0s    |
//! Note no other values than the ones specified should be used!
void gotoSleep(sleep_prescaler prescale = sleep_prescaler::sleep_8s);

//! Function that turns off unneeded interfaces and sets all pins to a low energy defined state
void saveEnergy();

//! Function that initializes timers 0, 1 and 2.
//! Timer 0 is used for micros()
//! Timer 1 is used for //! \todo fill this
//! Timer 2 is used for //! \todo fill this
void initTimers();

//! Inline function that initializes the ADC to a frequency of 125KHz
inline void initADC()
{
    // enable analog-digital converter
    // clock prescaler = 64, so the frequency is 125 KHz
    ADCSRA |= 0x86;
}

//! Function to read the battery voltage
//! \return long containing the voltage in mV
long getVCC();

//! Function that reads the battery voltage and smoothes it for \ref getSupplyVoltage
void checkVCC();

//! Function that returns the smoothed battery voltage
//! \return long containing the voltage in mV
long getSupplyVoltage();

//! Inline function that calculates the battery capacity percentage
//! \return uint8_t Battery capacity in percent
inline uint8_t getSupplyPercentage()
{
    return static_cast<uint8_t>((getSupplyVoltage() - 3000) / (12));
}

//! Function that checks the charger ic pins to determine if the node is charging
//! \return boolean true for charging and false for not charging
bool isCharging();

//! Function that checks the charger ic pins to determine if the node is charged
//! \return boolean true for charged and false for not charged
bool isCharged();

//! Function that returns the time since startup in ms
//! \return unsigned long ms
unsigned long millis();

//! Function that returns the time since startup in us
//! \return unsigned long us
unsigned long micros();

//! Function that delays the program for millis ms
//! \param millis Time to delay the program in ms
void delay(unsigned long millis);

//! Function that delays the program for micros us
//! \param micros Time to delay the program in us
void delayMicros(unsigned int micros);

//! Function that disables all interrupts
void noInterrupts();

//! Function that enables all interrupts
void interrupts();

//! Function that sets the pinmode of the specified pin
//! Possible modes are \ref INPUT, \ref OUTPUT and \ref INPUT_PULLUP
//! \param pin The pin to set the mode of
//! \param mode The mode to set the pin to
void pinMode(uint8_t pin, uint8_t mode);

//! Function that turns off pwm on the specified pin
//! This makes only sense for the specific pwm pins.
//! These are pins 3, 5, 6, 9, 10 and 11.
//! \param pin Pin for which to turn pwm off
void turnOffPWM(uint8_t pin);

//! Function to digitally control a pin \ref HIGH or \ref LOW
//! \param pin The pin to control
//! \param mode Boolean to set the pin \ref HIGH or \ref LOW
void digitalWrite(uint8_t pin, bool mode);

//! Function to digitally read a pin
//! \param pin The pin to read
//! \return Boolean true for \ref HIGH and false for \ref LOW
bool digitalRead(uint8_t pin);

//! Function to read an analog pin
//! \param pin The pin to read (14-21)
//! \return uint16_t ranging from 0 to 1024
uint16_t analogRead(uint8_t pin);

//! Function to write to an analog pin
//! \param pin The pin to write to (14-21)
//! \param val The value to write to the analog pin
void analogWrite(uint8_t pin, uint8_t val);

//! Function that blinks the status LED on pin 8 for 200ms
//!
//! \param isOn specify whether the led is already on or not
void blinkStatusLEDLong(bool isOn = false);

//! Function that blinks the status LED on pin 8 num times for 100ms
//!
//! \param num The number of blinks
//! \param isOn specify whether the led is already on or not
void blinkStatusLEDShort(uint8_t num, bool isOn = false);

const uint8_t TWI_STRT = 0;
const uint8_t TWI_DATA = 1;
const uint8_t TWI_ACK = 2;
const uint8_t TWI_STOP = 3;

//! Function that transmits twi commands like start, data, ack and stop
//! \param type The type of command (0-3)
//! \return uint8_t containing the masked value of the TWSR register
uint8_t twiTransmit(uint8_t type);

//! Function to read data from the twi EEPROM
//! \param address The start address to read from
//! \param data Pointer that will contain the read data
//! \param length Size to read from the EEPROM
//! \return int8_t containing the state of the operation
int8_t readEEPROM(uint16_t address, uint8_t* data, uint16_t length = 1);

//! Function to write data to the twi EEPROM
//! \param address The start address to write to
//! \param data Pointer that contains the data to be written
//! \param length Size to write to the EEPROM
//! \return int8_t containing the state of the operation
int8_t writeEEPROM(uint16_t address, uint8_t* data, uint16_t length = 1);

//! Function to update data in the twi EEPROM that will only write data packets if they don't match
//! \param address The start address to write to
//! \param data Pointer that contains the data to be written
//! \param length Size to write to the EEPROM
//! \return int8_t containing the state of the operation
int8_t updateEEPROM(uint16_t address, uint8_t* data, uint16_t length = 1);

#ifdef ENABLE_PRINT
//! Function that initializes the print functions and registers so data can be written to the USART0 interface
void initPrint();

//! Function that writes a char value to the USART0 interface
//! \param c char value to be written to USART0
void print(char c);

//! Function that writes a uint8_t value with a specific base of 8, 10 or 16 to the USART0 interface
//! \param i uint8_t value to be written to USART0
//! \param b The base in that i is written to USART0. Standard is 10
void print(uint8_t i, uint8_t b = 10);

//! Function that writes a int8_t value with a specific base of 8, 10 or 16 to the USART0 interface
//! \param i int8_t value to be written to USART0
//! \param b The base in that i is written to USART0. Standard is 10
void print(int8_t i, uint8_t b = 10);

//! Function that writes a uint16_t value with a specific base of 8, 10 or 16 to the USART0 interface
//! \param i uint16_t value to be written to USART0
//! \param b The base in that i is written to USART0. Standard is 10
void print(uint16_t i, uint8_t b = 10);

//! Function that writes a int16_t value with a specific base of 8, 10 or 16 to the USART0 interface
//! \param i int16_t value to be written to USART0
//! \param b The base in that i is written to USART0. Standard is 10
void print(int16_t i, uint8_t b = 10);

//! Function that writes a uint32_t value with a specific base of 8, 10 or 16 to the USART0 interface
//! \param i uint32_t value to be written to USART0
//! \param b The base in that i is written to USART0. Standard is 10
void print(uint32_t i, uint8_t b = 10);

//! Function that writes a int32_t value with a specific base of 8, 10 or 16 to the USART0 interface
//! \param i int32_t value to be written to USART0
//! \param b The base in that i is written to USART0. Standard is 10
void print(int32_t i, uint8_t b = 10);

//! Function that writes a double value to the USART0 interface
//! \param d double value to be written to USART0
void print(double d);

//! Function that writes a float value to the USART0 interface
//! \param f float value to be written to USART0
void print(float f);

//! Function that writes a const char pointer value to the USART0 interface
//! \param s const char pointer value to be written to USART0
void print(const char* s);

//! Function that writes a progmem string to the USART0 interface
//! \param pgmStr progmem string to be written to USART0
void print_P(PGM_P pgmStr);

//! Template that write a newline after a print()
//! \param val A value of any of the print() types
template <typename T>
void println(T val)
{
    print(val);
    print('\n');
}

//! Template that write a newline after a print() with base
//! \param val A value of any of the print() types
//! \param base The base in that val is written to USART0. Standard is 10
template <typename T>
void println(T val, uint8_t base)
{
    print(val, base);
    print('\n');
}

//! Inline function that writes a newline after a progmem string
//! \param pgmStr progmem string to be written to USART0
inline void println_P(PGM_P pgmStr)
{
    print_P(pgmStr);
    print('\n');
}

//! Inline function that writes a newline to USART0
inline void println()
{
    print('\n');
}
#else
inline void initPrint() {}
template <typename... Ignored>
inline void print(Ignored...)
{}
template <typename... Ignored>
inline void println(Ignored...)
{}
template <typename... Ignored>
inline void println_P(Ignored...)
{}
template <typename... Ignored>
inline void print_P(Ignored...)
{}
#endif

long randomMinMax(long min, long max);

//! Function that initializes the SPI interface
void spiInit();

//! Function that stops the SPI interface
void spiEnd();

//! Function that initializes a SPI transaction with configurations
//! \param settings pointer to 2 element array that contains the settings
void spiBeginTransaction(const uint8_t* settings);

//! Function that transfers and receives SPI data
//! \param data uint8_t that contains the data to be sent
//! \return uint8_t containing the received data
uint8_t spiTransfer(uint8_t data);

template <typename T>
inline T constrain(const T val, const T min, const T max)
{
    return (val > max) ? max : ((val < min) ? min : val);
}

template <typename T>
inline T max(const T v1, const T v2)
{
    return (v1 > v2) ? v1 : v2;
}

#endif //__COMMON_H__
