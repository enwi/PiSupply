/*
 * common.cpp
 *
 * Created: 18.08.2016 17:13:26
 * Author: Moritz Wirger, Jan Rogall
 */

#include "common.h"

#include <avr/common.h> // needed for SREG
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h> // needed for PROGMEM
#include <avr/sleep.h>
#include <util/twi.h> // obviously TWI

// used for easier debugging enabling
#ifdef DEBUG
#define IF_DEBUG(x) x
#else
#define IF_DEBUG(x) ;
#endif

void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();
    return;
}

volatile unsigned long timer0_num_overflow = 0;
volatile unsigned long timer0_millis = 0;
// contains micros >> 3 which are no full millis yet
volatile uint8_t timer0_fract = 0;

// micros counter interrupt (timer0 interrupt)
ISR(TIMER0_OVF_vect)
{
    // no reload, because it messes with PWM
    // TCNT0 = 131;		// reload
    // The interrupt is called every 16384 CPU ticks (256 * 64)
    // At 8MHz this is every 2ms 48us
    // At 16MHz this is every 1ms 24us
    // Copy values to prevent read operations on every access
    unsigned long m = timer0_millis;
    uint8_t f = timer0_fract;

#if F_CPU == 8000000UL || F_CPU == 1000000UL
    // increase millis by 2
    m += 2;
    // increase fractional part by 48>>3
    f += 6;
    // if fract is bigger than 1 ms
    if (f >= 125)
    {
        f -= 125;
        ++m;
    }
#elif F_CPU == 16000000UL
    // increase millis by 1
    m += 1;
    // increase fractional part by 24>>3
    f += 3;
    // if fract is bigger than 1 ms
    if (f >= 125)
    {
        f -= 125;
        ++m;
    }
#endif

    timer0_fract = f;
    timer0_millis = m;
    // Increase overflow timer for micros() function
    ++timer0_num_overflow;
}

volatile uint16_t WDT_INTS = 0;
// handles the Watchdog Time-out Interrupt
ISR(WDT_vect)
{
    ++WDT_INTS;
}

// variable to store interrupt function
void (*interrupt0func)(void);

// interrupt 0
ISR(INT0_vect)
{
    cli();
    interrupt0func();
    sei();
}

void attachInterrupt0(void (*func)())
{
    /*interrupt0func = func;

    pinMode(2, INPUT_PULLUP);

    EICRA = (EICRA & 0xFC) | 0x01;
    EIMSK |= 0x01;

    sei();              // turn on interrupts*/
}

void detachInterrupt0()
{
    /*EIMSK &= 0xFE;
    interrupt0func = NULL;
    */
}

void initWDT(sleep_prescaler prescale)
{
    wdt_reset();
    MCUSR &= ~(1 << WDRF); // clear WDRF
    WDTCR |= (1 << WDCE) | (1 << WDE); // enable WDTCSR change
    WDTCR = (1 << WDIE) | static_cast<uint8_t>(prescale); // set duration according prescale
}

void gotoSleep(sleep_prescaler prescale)
{
    initWDT(prescale);
    uint8_t adcsra = ADCSRA; // save the ADC Control and Status Register A
    ADCSRA = 0; // disable the ADC
    ADCSRB &= ~(_BV(ACME));
    ACSR &= (1 << ACIE); // disable Analog Comparator interrupt to not trigger it in the next command
    ACSR |= (1 << ACD); // disable Analog Comparator
    // PRR |= 0x08;				// Power Reduction Register, can't disable Timer/Counter0 and Analog Comp resulting in node
    // not functioning DIDR0 &= 0b11111111;		// disable ADC0D - ADC7D
    noInterrupts(); // timed sequences follow
    // timer0_millis += 8000;		//! \todo move this somewhere else!!!
    // EIFR = (1<<INTF0);  		// clear flag for interrupt 0
    // EIFR = (1<<INTF1);  		// clear flag for interrupt 1
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    // set_sleep_mode(SLEEP_MODE_IDLE);
    // WDT_interrupt = false;
    sleep_enable();
    uint8_t mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE); // turn off the brown-out detector while sleeping
    uint8_t mcucr2
        = mcucr1 & ~_BV(BODSE); // BODS stays active for 3 cycles, sleep instruction must be executed while it's active
    MCUCR = mcucr1;
    MCUCR = mcucr2;
    interrupts(); // need interrupts now
    sleep_cpu(); // go to sleep
    sleep_disable(); // wake up here
    ADCSRA = adcsra; // restore ADCSRA (enable ADC)
    ADCSRB &= (_BV(ACME)); // restore ADCSRB (enable Analog Comparator)
    ACSR &= ~(1 << ACD);
    // PRR &= ~ 0x08;         	// reenable PRR
    // DIDR0 &= 0b00000000;  // reenable ADC0D - ADC7D //! \todo leave uneeded turned off
}

//! \todo consider activation of digital buffers of analog pins and digital pins 6 & 7 when needed as digital pin
void saveEnergy()
{
    uint8_t oldSREG = SREG;
    cli();
    PRR |= 0x0A; // shut off USI and timer1

    DDRB &= ~0x3F; // except pin
    PORTB |= 0x3F; // except pin

    DIDR0 = 0x16; // disable ADC1D, ADC2D and AIN1D to save more energy
    SREG = oldSREG;
}

void initTimers()
{
    // Timer 0:
#if F_CPU == 8000000UL || F_CPU == 16000000UL
    // timer 0 src = system clock, prescaler 64
    TCCR0B |= 0x03;
#elif F_CPU == 1000000UL
    // timer 0 src = system clock, prescaler 8
    TCCR0B |= 0x02;
#endif

    // TCNT0 = 131;		// preload timeout
    // Enable overflow interrupt
    TIMSK |= 0x02;
    /*
    // Timer 1:
    // timer 1 src = system clock, prescaler 64
    // timer 1 noise canceler: disabled (default)
    // timer 1 edge select: falling (default)
    // timer 1 WGM13-WGM12: 00
    TCCR1 = 0x07;
    // timer 1 WGM11-WGM10: 01
    // -> timer 1 mode = PGM, Phase correct, 8 bit
    TCCR1A |= 0x01;

    // Timer 2:
    // timer 2 src = system clock, prescaler 64
    TCCR2B |= 0x04;
    // timer 2 mode = PGM, Phase correct
    TCCR2A |= 0x01;*/
    sei(); // don't forget to activate interrupts or this won't work
}

long getVCC()
{
// Read 1.1V reference against AVcc
// set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC))
        ; // measuring

    // uint8_t low  = ADCL;				// must read ADCL first - it then locks ADCH
    // uint8_t high = ADCH;				// unlocks both

    // long result = (high<<8) | low;	// usage of ADCW saves us 8 Byte

    return (1130000L / ADCW); // 1142664L/result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
}

unsigned long lastVccCheck = 0;
long _supplyVoltages[3] = {};

void checkVCC()
{
    if (WDT_INTS % 2)
    {
        if (isCharging())
        {
            blinkStatusLEDLong();
        }
        _supplyVoltages[1] = getVCC();
        delay(500);
        _supplyVoltages[2] = getVCC();
        _supplyVoltages[0] = ((_supplyVoltages[1] + _supplyVoltages[1] + _supplyVoltages[2] + _supplyVoltages[2]) >> 2);
        lastVccCheck = millis();
    }
}

long getSupplyVoltage()
{
    return _supplyVoltages[0];
}

// CHRG = A6 = 20
// STDBY = A7 = 21
// charging = CHRG(LOW) & STDBY(HIGH)
bool isCharging()
{
    if (analogRead(20) < 512 && analogRead(21) > 511)
    {
        return true;
    }
    return false;
}

// charged  = CHRG(HIGH) & STDBY(LOW)
bool isCharged()
{
    if (analogRead(20) > 511 && analogRead(21) < 512)
    {
        return true;
    }
    return false;
}

unsigned long millis()
{
    uint8_t oldSREG = SREG;
    cli();
    unsigned long m = timer0_millis;
    SREG = oldSREG;
    return m;
}

unsigned long micros()
{
    unsigned long m;
    uint8_t t;
    uint8_t oldSREG = SREG;
    cli();
    m = timer0_num_overflow;
    t = TCNT0;
    // If an overflow occurred and was not handled yet
    if ((TIFR & 0x02) && (t < 255))
    {
        m++;
    }
    SREG = oldSREG;
#if F_CPU == 16000000UL
    // Return the combined value from overflows and ticks * 64 / (4 ticks/us) (16MHz)
    return ((m << 8) + t) * 4;
#else
    // Return the combined value from overflows and ticks * 64 / (8 ticks/us) (8MHz)
    return ((m << 8) + t) * 8;
#endif
}

void delay(unsigned long millis)
{
    uint16_t start = (uint16_t)micros();
    while (millis > 0)
    {
        ;
        if (((uint16_t)micros() - start) >= 1000)
        {
            millis--;
            start += 1000;
        }
    }
}

void delayMicros(unsigned int micros)
{
    // call = 4 cycles + 2 to 4 cycles to init us(2 for constant delay, 4 for variable)

    // calling avrlib's delay_us() function with low values (e.g. 1 or
    // 2 microseconds) gives delays longer than desired.
    // delay_us(us);

#if F_CPU >= 16000000L
    // for the 16 MHz clock on most Arduino boards

    // for a one-microsecond delay, simply return.  the overhead
    // of the function call yields a delay of approximately 1 1/8 us.
    if (--micros == 0)
        return;

    // the following loop takes a quarter of a microsecond (4 cycles)
    // per iteration, so execute it four times for each microsecond of
    // delay requested.
    micros <<= 2;

    // account for the time taken in the preceeding commands.
    micros -= 2;
#else
    // for the 8 MHz internal clock on the ATmega168

    // for a one- or two-microsecond delay, simply return.  the overhead of
    // the function calls takes more than two microseconds.  can't just
    // subtract two, since us is unsigned; we'd overflow.
    if (--micros == 0)
        return;
    if (--micros == 0)
        return;

    // the following loop takes half of a microsecond (4 cycles)
    // per iteration, so execute it twice for each microsecond of
    // delay requested.
    micros <<= 1;

    // partially compensate for the time taken by the preceeding commands.
    // we can't subtract any more than this or we'd overflow w/ small delays.
    micros--;
#endif

    // busy wait
    __asm__ __volatile__("1: sbiw %0,1"
                         "\n\t" // 2 cycles
                         "brne 1b"
                         : "=w"(micros)
                         : "0"(micros) // 2 cycles
    );
    // return = 4 cycles
}

uint8_t cSREG;

void noInterrupts()
{
    cSREG = SREG;
    cli();
}

void interrupts()
{
    SREG = cSREG;
}

// keep in mind that ADC6 & ADC7 do not have digital input buffers
void pinMode(uint8_t pin, uint8_t mode)
{
    MicroPin::PWMPin {pin}.pinMode(mode);
}

void turnOffPWM(uint8_t pin)
{
    MicroPin::PWMPin {pin}.clearPWM();
}

void digitalWrite(uint8_t pin, bool mode)
{
    MicroPin::PWMPin {pin}.operator=(mode);
}

bool digitalRead(uint8_t pin)
{
    return static_cast<bool>(MicroPin::PWMPin(pin));
}

uint16_t analogRead(uint8_t pin)
{
    return MicroPin::AnalogPin(pin - 14).analogRead();
}

void analogWrite(uint8_t pin, uint8_t val)
{
    MicroPin::PWMPin(pin).analogWrite(val);
}

void blinkStatusLEDLong(bool isOn)
{
    statusLED = !isOn;
    delay(500);
    statusLED = isOn;
}

void blinkStatusLEDShort(uint8_t num, bool isOn)
{
    for (uint8_t i = 0; i < num; ++i)
    {
        if (i != 0)
        {
            delay(100);
        }
        statusLED = !isOn;
        delay(100);
        statusLED = isOn;
    }
}

// Implementations of operator new
void* operator new(size_t size)
{
    return malloc(size);
}
void operator delete(void* ptr)
{
    free(ptr);
}
void operator delete(void* ptr, size_t size)
{
    free(ptr);
}
void* operator new[](size_t size)
{
    return malloc(size);
}

void operator delete[](void* ptr)
{
    free(ptr);
}
void operator delete[](void* ptr, size_t size)
{
    free(ptr);
}
// Placement new for stl
#include <pnew.cpp>