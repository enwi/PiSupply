/*
 * Pin.h
 *
 * Created: 28.07.2017 11:11:48
 *  Author: Jan
 */

#ifndef PIN_H_
#define PIN_H_

#include <avr/common.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>

constexpr const uint8_t INPUT = 0; //!< constant to set a pin of \ref Pins to input mode
constexpr const uint8_t OUTPUT = 1; //!< constant to set a pin of \ref Pins to output mode
constexpr const uint8_t INPUT_PULLUP = 2; //!< constant to set a pin of \ref Pins to input pullup mode

// constexpr const bool LOW = false;			//!< constant to set or read a pin of \ref Pins as low
// constexpr const bool HIGH = true;			//!< constant to set or read a pin of \ref Pins as high

namespace detail
{
    extern const uint8_t pin_bitmask[22] PROGMEM;
    extern const uint8_t pin_timer[22] PROGMEM;

    inline uint8_t GetRuntimePinBitmask(uint8_t pin) { return (uint8_t)pgm_read_byte(pin_bitmask + pin); }
    inline uint8_t GetRuntimePinTimer(uint8_t pin) { return (uint8_t)pgm_read_byte(pin_timer + pin); }
    constexpr uint8_t GetPinPortN(uint8_t pin)
    {
        // pin  0 - 7 -> port 0
        // pin  8 - 13 -> port 1
        // pin A0 - A7 (14 - 21) -> port 2
        return 0;
    }
    constexpr volatile uint8_t* GetPortType(uint8_t port) { return &DDRB; }
    constexpr volatile uint8_t* GetPortPort(uint8_t port) { return &PORTB; }
    constexpr volatile uint8_t* GetPortData(uint8_t port) { return &PINB; }
    constexpr volatile uint8_t* GetPinType(uint8_t pin) { return GetPortType(GetPinPortN(pin)); }
    constexpr volatile uint8_t* GetPinPort(uint8_t pin) { return GetPortPort(GetPinPortN(pin)); }
    constexpr volatile uint8_t* GetPinData(uint8_t pin) { return GetPortData(GetPinPortN(pin)); }

    template <uint8_t b, uint8_t t, bool analog = false, bool digital = true>
    struct PinTraitsHelper
    {
        static constexpr bool exists = true;
        static constexpr bool hasTimer = (t != 0);
        static constexpr bool isAnalog = analog;
        static constexpr bool hasDigital = digital;
        static constexpr uint8_t bitmask = b;
        static constexpr uint8_t timer = t;
    };
    template <uint8_t num>
    struct PinTraits
    {
        static constexpr bool exists = false;
    };
    // Port D
    template <>
    struct PinTraits<0> : PinTraitsHelper<0x01, 0, true>
    {};
    template <>
    struct PinTraits<1> : PinTraitsHelper<0x02, 0, true>
    {};
    template <>
    struct PinTraits<2> : PinTraitsHelper<0x04, 0, true>
    {};
    template <>
    struct PinTraits<3> : PinTraitsHelper<0x08, 6, true>
    {};
    template <>
    struct PinTraits<4> : PinTraitsHelper<0x16, 0, true>
    {};
    template <>
    struct PinTraits<5> : PinTraitsHelper<0x32, 2, true>
    {};
} // namespace detail

template <uint8_t num>
class Port
{
public:
    static constexpr volatile uint8_t* GetTypeReg() { return detail::GetPortType(num); }
    static constexpr volatile uint8_t* GetPortReg() { return detail::GetPortPort(num); }
    static constexpr volatile uint8_t* GetDataReg() { return detail::GetPortData(num); }
};

namespace Utils
{
    inline void pinMode(volatile uint8_t* typeReg, volatile uint8_t* portReg, uint8_t bitmask, uint8_t mode)
    {
        uint8_t oldSREG = SREG;
        cli();
        if ((mode & 0x01) == 0)
        {
            // Set mode to input
            *typeReg &= ~bitmask;
        }
        else
        {
            // Set mode to output
            *typeReg |= bitmask;
        }
        if ((mode & 0x02) == 0)
        {
            // Write low/disable pullup
            *portReg &= ~bitmask;
        }
        else
        {
            // Write high/enable pullup
            *portReg |= bitmask;
        }
        SREG = oldSREG;
    }
    namespace detail
    {
        inline void internalDigitalWriteOn(volatile uint8_t* portReg, uint8_t bitmask) { *portReg |= bitmask; }
        inline void internalDigitalWriteOff(volatile uint8_t* portReg, uint8_t bitmask) { *portReg &= ~bitmask; }
    } // namespace detail
    inline void digitalWrite(volatile uint8_t* portReg, uint8_t bitmask, bool on)
    {
        // detail::internalDigitalWriteOn/Off can be used directly if portReg, bitmask and on are all known at compile
        // time
        uint8_t oldSREG = SREG;
        cli();
        if (on)
        {
            detail::internalDigitalWriteOn(portReg, bitmask);
        }
        else
        {
            detail::internalDigitalWriteOff(portReg, bitmask);
        }
        SREG = oldSREG;
    }
    inline bool digitalRead(volatile uint8_t* dataReg, uint8_t bitmask) { return *dataReg & bitmask; }
    inline void clearPWM(uint8_t timerNum)
    {
        switch (timerNum)
        {
        case 1:
            _SFR_BYTE(TCCR0A) &= ~_BV(COM0A1);
            break;
        case 2:
            _SFR_BYTE(TCCR0A) &= ~_BV(COM0B1);
            break;
        case 3:
            _SFR_BYTE(TCCR1) &= ~_BV(COM1A1);
            break;
        case 4:
            _SFR_BYTE(GTCCR) &= ~_BV(COM1B1);
            break;
        }
    }
    inline void analogWrite(uint8_t timerNum, uint8_t val)
    {
        // 6	{&TCCR2A, &OCR2B, COM2B1},	// pin  3
        // 2	{&TCCR0A, &OCR0B, COM0B1},	// pin  5
        // 1	{&TCCR0A, &OCR0A, COM0A1},	// pin  6
        // 3	{&TCCR1A, &OCR1A, COM1A1},	// pin  9
        // 4	{&TCCR1A, &OCR1B, COM1B1},	// pin 10
        // 5	{&TCCR2A, &OCR2A, COM2A1},	// pin 11
        switch (timerNum)
        {
        case 1:
            _SFR_BYTE(TCCR0A) |= _BV(COM0A1);
            OCR0A = val;
            break;
        case 2:
            _SFR_BYTE(TCCR0A) |= _BV(COM0B1);
            OCR0B = val;
            break;
        case 3:
            _SFR_BYTE(TCCR1) |= _BV(COM1A1);
            OCR1A = val;
            break;
        case 4:
            _SFR_BYTE(GTCCR) |= _BV(COM1B1);
            OCR1B = val;
            break;
        }
    }
    // analogPin = analog pin number (0-7), not actual pin number
    inline uint16_t analogRead(uint8_t analogPin)
    {
        ADMUX = (ADMUX & 0xF0) | (analogPin & 0x0F);
        // Start conversion
        ADCSRA |= _BV(ADSC);
        // Wait
        while (bit_is_set(ADCSRA, ADSC))
            ;
        return ADCW;
    }
} // namespace Utils

// Types for high and low, for compile time optimization if available
struct High_type
{
    constexpr operator bool() const { return true; }
};
struct Low_type
{
    constexpr operator bool() const { return false; }
};
// HIGH is convertible to true, but use it instead for more optimizations
constexpr High_type HIGH;
// LOW is convertible to false, but use it instead for more optimizations
constexpr Low_type LOW;

template <uint8_t num>
class StaticDigitalPin
{
private:
    using PinTraits = detail::PinTraits<num>;

public:
    constexpr StaticDigitalPin() = default;
    static_assert(detail::PinTraits<num>::exists, "Pin number does not exist");
    using PortType = Port<detail::GetPinPortN(num)>;
    void pinMode(uint8_t mode) const
    {
        Utils::pinMode(PortType::GetTypeReg(), PortType::GetPortReg(), PinTraits::bitmask, mode);
    }
    void operator=(bool on) const
    {
        static_assert(PinTraits::hasDigital, "Cannot digital write on pin without digital buffers");
        Utils::digitalWrite(PortType::GetPortReg(), PinTraits::bitmask, on);
    }
    void operator=(High_type) const
    {
        static_assert(PinTraits::hasDigital, "Cannot digital write on pin without digital buffers");
        // Directly turn on, without disabling interrupts as all values are constexpr
        Utils::detail::internalDigitalWriteOn(PortType::GetPortReg(), PinTraits::bitmask);
    }
    void operator=(Low_type) const
    {
        static_assert(PinTraits::hasDigital, "Cannot digital write on pin without digital buffers");
        // Directly turn off, without disabling interrupts as all values are constexpr
        Utils::detail::internalDigitalWriteOff(PortType::GetPortReg(), PinTraits::bitmask);
    }
    operator bool() const
    {
        static_assert(PinTraits::hasDigital, "Cannot digital read on pin without digital buffers");
        return Utils::digitalRead(PortType::GetDataReg(), PinTraits::bitmask);
    }
};
template <uint8_t num>
class StaticPWMPin : public StaticDigitalPin<num>
{
private:
    using PinTraits = detail::PinTraits<num>;
    using Base = StaticDigitalPin<num>;

public:
    static_assert(PinTraits::hasTimer, "Pin does not have timer, cannot use StaticPWMPin");
    void operator=(bool on) const
    {
        clearPWM();
        Base::operator=(on);
    }
    void operator=(High_type) const
    {
        clearPWM();
        Base::operator=(High_type {});
    }
    void operator=(Low_type) const
    {
        clearPWM();
        Base::operator=(Low_type {});
    }
    operator bool() const
    {
        clearPWM();
        return Base::operator bool();
    }
    void analogWrite(uint8_t val) const
    {
        if (val == 0)
        {
            *this = 0;
        }
        else if (val == 255)
        {
            *this = 255;
        }
        else
        {
            Utils::analogWrite(PinTraits::timer, val);
        }
    }
    void clearPWM() const { Utils::clearPWM(PinTraits::timer); }
};
template <uint8_t num>
class StaticAnalogPin
{
private:
    using PinTraits = detail::PinTraits<num>;

public:
    static_assert(PinTraits::isAnalog, "Pin is not an analog input, cannot use StaticAnalogPin");
    constexpr StaticAnalogPin() = default;
    uint16_t analogRead() const
    {
        // TODO: Remove this constant
        return Utils::analogRead(num - 14);
    }
};
template <uint8_t num>
class StaticAnalogDigitalPin : public StaticDigitalPin<num>, public StaticAnalogPin<num>
{};
// There is no analog input pin with PWM

class DigitalPin
{
public:
    explicit DigitalPin(uint8_t num) : num(num) {}
    void pinMode(uint8_t mode) const
    {
        Utils::pinMode(detail::GetPinType(num), detail::GetPinPort(num), detail::GetRuntimePinBitmask(num), mode);
    }
    void operator=(bool on) const
    {
        Utils::digitalWrite(detail::GetPinPort(num), detail::GetRuntimePinBitmask(num), on);
    }
    operator bool() const { return Utils::digitalRead(detail::GetPinData(num), detail::GetRuntimePinBitmask(num)); }
    uint8_t GetNum() const { return num; }

private:
    uint8_t num;
};

class PWMPin : public DigitalPin
{
public:
    explicit PWMPin(uint8_t num) : DigitalPin(num) {}
    void operator=(bool on) const
    {
        clearPWM();
        DigitalPin::operator=(on);
    }
    operator bool() const
    {
        clearPWM();
        return DigitalPin::operator bool();
    }
    void analogWrite(uint8_t val) const
    {
        if (val == 0)
        {
            *this = 0;
        }
        else if (val == 255)
        {
            *this = 255;
        }
        else
        {
            const uint8_t timerNum = detail::GetRuntimePinTimer(GetNum());
            if (timerNum != 0)
            {
                Utils::analogWrite(timerNum, val);
            }
            else
            {
                // Set to closer value
                DigitalPin::operator=(val >= 128);
            }
        }
    }
    void clearPWM() const
    {
        uint8_t timerNum = detail::GetRuntimePinTimer(GetNum());
        if (timerNum != 0)
        {
            Utils::clearPWM(timerNum);
        }
    }
};
class AnalogPin
{
public:
    // analogPinNum (0-7) != pinNum!!!
    explicit AnalogPin(uint8_t analogPinNum) : analogPinNum(analogPinNum) {}
    uint16_t analogRead() const { return Utils::analogRead(analogPinNum); }

private:
    uint8_t analogPinNum;
};
class AnalogDigitalPin : public DigitalPin
{
    // Reimplemented AnalogPin's functionality, because otherwise there would be 1 byte overhead
public:
    explicit AnalogDigitalPin(uint8_t num) : DigitalPin(num) {}
    uint16_t analogRead() const
    {
        // TODO: get rid of this constant
        return Utils::analogRead(GetNum() - 14);
    }
};

#endif /* PIN_H_ */