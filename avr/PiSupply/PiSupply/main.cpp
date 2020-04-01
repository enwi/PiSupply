/*
 * PiSupply.cpp
 *
 * Created: 11.02.2018 11:23:11
 * Author : Schnittchen
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h> // watchdog sleep
#include <avr/wdt.h> // watchdog sleep
#include <common.h>

#include "fCpu.h"

enum class Action
{
    none = 0,
    shutoff = 0x01,
    reset = 0x80
};

constexpr uint8_t short_press = 2; //!< The time in seconds to count as short press
constexpr uint8_t long_press = 10; //!< The time in seconds to count as long press

constexpr uint16_t SHUTOFF_DELAY = 5500; //!< The time to wait before turning off power after pi has shut down

constexpr uint16_t SHUTOFF_TIMEOUT = 20000; //!< The maximum time we wait for the raspberry pi to shutdown
constexpr uint8_t MAX_SHUTOFF_RETRIES
    = 2; //!< The maximum amount of retries to turn off the pi until we cut power
uint8_t tries_to_shutoff = 0; //!< Number of tries we already had
constexpr MicroPin::StaticDigitalPin<0>
    input_button; //!< Button the user can press to safely turn on/off or long press to cut power
constexpr MicroPin::StaticDigitalPin<1> ctrl_mos; //!< Control line to either turn on/off power to pi and peripherals
constexpr MicroPin::StaticDigitalPin<3> state_pi; //!< State of the pi: HIGH when booted, LOW when off
constexpr MicroPin::StaticDigitalPin<4> ctrl_pi; //!< Control line to tell pi to turn off when HIGH

volatile bool button_interrupt_flag = false;

enum class PowerState
{
    off = 0,
    on,
    shutdown
};
//! Current state of PiSupply
PowerState state = PowerState::off;

enum class BatteryState
{
    unknown = 0,
    full,
    low
};
//! Current state of Battery
BatteryState bat_state = BatteryState::unknown;

enum class ButtonType
{
    unkown = 0,
    button,
    switchButton
};
//! Type of the button
ButtonType button_type = ButtonType::unkown;

ISR(PCINT0_vect)
{
    button_interrupt_flag = true; // set interrupt flag to true
}

bool shutOffPi()
{
    bool timeout = false;
    if (state_pi
        && state == PowerState::shutdown) // only attempt to shut down, when pi is alive and we want to shutdown
    {
        ctrl_pi = MicroPin::high;
        uint32_t start = millis();
        uint32_t last = 0;
        bool helper = false;
        while (state_pi && !timeout)
        {
            uint32_t now = millis();
            if (now >= last + 500)
            {
                statusLED = helper;
                helper = !helper;
                last = now;
            }
            timeout = millis() >= (start + SHUTOFF_TIMEOUT);
            delay(200); // don't poll the pin to often
        }
        return !timeout;
    }
    else
    {
        return false; // pi was never alive
    }
}

void userShutOff()
{
    blinkStatusLEDLong(true);
    if (shutOffPi()) // if we could safely shut off
    {
        statusLED = MicroPin::high;
        delay(SHUTOFF_DELAY);
        ctrl_mos = MicroPin::low;
        ctrl_pi = MicroPin::low;
        statusLED = MicroPin::low;
        state = PowerState::off;
    }
    else
    {
        blinkStatusLEDShort(4, true);
        statusLED = MicroPin::high;
        state = PowerState::on;
    }
}

ButtonType testButtonType()
{
    gotoSleep(sleep_prescaler::sleep_250ms);
    uint8_t button_reading = input_button;
    gotoSleep(sleep_prescaler::sleep_125ms);
    button_reading += input_button;
    gotoSleep(sleep_prescaler::sleep_125ms);
    button_reading += input_button;
    gotoSleep(sleep_prescaler::sleep_125ms);
    button_reading += input_button;
    if (button_reading / 4)
    {
        // Is it really needed to tell the user if he connected a button?
        // blinkStatusLEDShort(1);
        // statusLED = HIGH;
        return ButtonType::button;
    }
    else
    {
        // Is it really needed to tell the user if he connected a switch?
        // blinkStatusLEDShort(4);
        //  statusLED = HIGH;
        return ButtonType::switchButton;
    }
}

Action getButtonAction()
{
    Action action = Action::none;
    unsigned long start_time = millis();
    unsigned long time_delta = 0;
    while (!input_button)
    {
        time_delta = (millis() - start_time);
        if (time_delta >= (uint16_t)short_press * 1000)
        {
            if (action == Action::none)
            {
                blinkStatusLEDShort(1, true);
            }
            action = Action::shutoff;
        }
        if (time_delta >= (uint16_t)long_press * 1000)
        {
            action = Action::reset;
            break;
        }
        delay(200);
    }
    return action;
}

Action getSwitchAction()
{
    unsigned long start_time = millis();
    while (input_button)
    {
        unsigned long time_delta = (millis() - start_time);
        if (time_delta >= short_press * 1000)
        {
            return Action::shutoff;
        }
        delay(200);
    }
    return Action::none;
}

void init()
{
    // saveEnergy();				// turn off unneeded stuff
    initADC(); // initialize ADC for analog pins
    initTimers(); // init millis/micros
    ctrl_mos.pinMode(MicroPin::output);
    ctrl_mos = MicroPin::low;
    ctrl_pi.pinMode(MicroPin::output);
    ctrl_pi = MicroPin::low;
    state_pi.pinMode(MicroPin::input);
    input_button.pinMode(MicroPin::inputPullup);
    statusLED.pinMode(MicroPin::output);

    if (getVCC() <= 3200) // check battery state
    {
        bat_state = BatteryState::low;
    }
    else
    {
        bat_state = BatteryState::full;
    }

    GIMSK |= (1 << PCIE); // enable pin change interrupts
    PCMSK |= (1 << PCINT0); // enable pin 0 for PCINT
}

int main(void)
{
    init();
    while (true)
    {
        if (button_type == ButtonType::switchButton)
        {
            if ((state == PowerState::on && input_button) || (state == PowerState::off && !input_button))
            {
                button_interrupt_flag = true;
            }
        }
        if (button_interrupt_flag && bat_state == BatteryState::full)
        {
            PCMSK &= ~(1 << PCINT0); // disable PCINT0

            if (state == PowerState::off)
            {
                statusLED = MicroPin::high;
                ctrl_mos = MicroPin::high;
                state = PowerState::on;

                if (button_type == ButtonType::unkown)
                {
                    button_type = testButtonType();
                }
            }
            else
            {
                Action action = Action::none;
                if (button_type == ButtonType::button)
                {
                    action = getButtonAction();
                }
                else if (button_type == ButtonType::switchButton)
                {
                    action = getSwitchAction();
                }

                if (action == Action::shutoff)
                {
                    state = PowerState::shutdown;
                    userShutOff();
                    // statusLED = LOW;		// only test code
                    // state = state_off;	// only test code
                }
                else if (action == Action::reset)
                {
                    statusLED = MicroPin::low;
                    ctrl_mos = MicroPin::low; // kill pi
                    state = PowerState::off;
                    gotoSleep(sleep_prescaler::sleep_4s); // don't retrigger
                }
            }
        }
        long vcc = getVCC();
        if (vcc <= 3200) // try to turn off Pi, when battery is low
        {
            bat_state = BatteryState::low;
            if (state == PowerState::off)
            {
                blinkStatusLEDLong(false);
            }
            else
            {
                blinkStatusLEDLong(true);
                state = PowerState::shutdown;
                if (shutOffPi()
                    || (tries_to_shutoff
                        >= MAX_SHUTOFF_RETRIES)) // shut off either when Pi is shutting down or if it does not respond
                {
                    delay(SHUTOFF_DELAY);
                    ctrl_mos = MicroPin::low;
                    ctrl_pi = MicroPin::low;
                    statusLED = MicroPin::low;
                    tries_to_shutoff = 0; // successfully shut off, so reset variable
                    state = PowerState::off;
                }
                else
                {
                    statusLED = MicroPin::high;
                    blinkStatusLEDShort(2, true);
                    ++tries_to_shutoff; // could not turn off, so keep track of it
                }
            }
        }
        else if (vcc >= 3500 && bat_state == BatteryState::low)
        {
            bat_state = BatteryState::full;
        }
        if (button_interrupt_flag)
        {
            button_interrupt_flag = false;
            PCMSK |= (1 << PCINT0); // enable PCINT0
        }
        gotoSleep(sleep_prescaler::sleep_8s);
    }
}
