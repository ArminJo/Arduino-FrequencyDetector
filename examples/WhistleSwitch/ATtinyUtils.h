/*
 * ATtinyUtils.h
 *
 *  Copyright (C) 2018-2020  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ArduinoUtils https://github.com/ArminJo/ArduinoUtils.
 *
 *  ArduinoUtils is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

//
// ATMEL ATTINY85
//
//                                         +-\/-+
//        PCINT5/!RESET/ADC0/dW (D5) PB5  1|    |8  Vcc
// PCINT3/XTAL1/CLKI/!OC1B/ADC3 (D3) PB3  2|    |7  PB2 (D2) SCK/USCK/SCL/ADC1/T0/INT0/PCINT2
//  PCINT4/XTAL2/CLKO/OC1B/ADC2 (D4) PB4  3|    |6  PB1 (D1) MISO/DO/AIN1/OC0B/OC1A/PCINT1 / TX Debug output
//                                   GND  4|    |5  PB0 (D0) MOSI/DI/SDA/AIN0/OC0A/!OC1A/AREF/PCINT0
//                                         +----+
#ifndef ATTINYUTILS_H_
#define ATTINYUTILS_H_

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)

#include <Arduino.h>
#include <avr/io.h>

#if defined(ARDUINO_AVR_DIGISPARK)
#undef LED_BUILTIN
#define LED_BUILTIN PB1
#elif defined(ARDUINO_AVR_DIGISPARKPRO)
// On a Digispark Pro we have PB1 / D9 / PCB pin 1
#undef LED_BUILTIN
#define LED_BUILTIN (9)
#endif



#if (F_CPU == 1000000)
#define TIMER0_CLOCK_DIVIDER_FOR_64_MICROS ((1 << CS01) | (1 << CS00))

#define TIMER1_CLOCK_DIVIDER_FOR_8_MICROS (1 << CS12)
#define TIMER1_CLOCK_DIVIDER_FOR_4_MICROS ((1 << CS11) | (1 << CS10))
#define TIMER1_CLOCK_DIVIDER_FOR_2_MICROS (1 << CS11)
#define TIMER1_CLOCK_DIVIDER_FOR_1_MICRO (1 << CS10)
#endif

#if (F_CPU == 8000000)
#define TIMER0_CLOCK_DIVIDER_FOR_128_MICROS ((1 << CS02) | (1 << CS00))

#define TIMER1_CLOCK_DIVIDER_FOR_8_MICROS ((1 << CS12) | (1 << CS11)| (1 << CS10))
#define TIMER1_CLOCK_DIVIDER_FOR_4_MICROS ((1 << CS12) | (1 << CS11))
#define TIMER1_CLOCK_DIVIDER_FOR_2_MICROS ((1 << CS12) | (1 << CS10))
#define TIMER1_CLOCK_DIVIDER_FOR_1_MICRO (1 << CS12)
#endif

/*
 * Only suitable for constant values
 * Loading of value adds 2 extra cycles (check .lss file for exact timing)
 *
 * Only multiple of 4 cycles are possible. Last loop is only 3 cycles.
 * 1 -> 3(+2) cycles
 * 2 -> 7(+2) cycles
 * 3 -> 11(+2) cycles
 * 4 -> 15(+2) cycles
 * 5 -> 19(+2) cycles
 * 6 -> 23(+2) cycles
 */
inline void delay4CyclesInlineExact(uint16_t a4Microseconds) {
    // the following loop takes 4 cycles (4 microseconds  at 1 MHz) per iteration
    __asm__ __volatile__ (
            "1: sbiw %0,1" "\n\t"    // 2 cycles
            "brne .-4" : "=w" (a4Microseconds) : "0" (a4Microseconds)// 2 cycles
    );
}
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
void PWMtone(unsigned int aFrequency, unsigned int aDurationMillis = 0);
void toneWithTimer1PWM(uint16_t aFrequency, bool aUseOutputB = false);
#endif

uint8_t getBODLevelFuses();
bool isBODLevelBelow2_7Volt();
bool isBODSFlagExistent();
void changeDigisparkClock();

#endif //  defined (__AVR_ATtiny85__)
#endif /* ATTINYUTILS_H_ */

#pragma once
