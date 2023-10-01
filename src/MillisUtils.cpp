/*
 *  MillisUtils.cpp
 *
 *  Unifies millis() timer handling for Digispark, AttinyCore and Arduino cores.
 *  - Start, stop and modify milliseconds timer and value.
 *  - Functions to compensate millis() timer value after long lasting ISR etc..
 *  - Blocking delayMilliseconds() function for use in noInterrupts context like ISR.
 *
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-Utils https://github.com/ArminJo/Arduino-Utils.
 *
 *  ArduinoUtils is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>
#include "MillisUtils.h"

#if defined(__AVR__)

#if !defined(cbi)
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#if !defined(sbi)
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void delayAndCallFunctionEveryMillis(unsigned int aDelayMillis, void (*aDelayCallback)(void)) {
    uint32_t tStartMillis = millis();
    do {
        if (aDelayCallback != NULL) {
            aDelayCallback();
        }
        delay(1);
    } while (millis() - tStartMillis <= aDelayMillis);
}

/*
 *
 */
void addToMillis(uint16_t aMillisToAdd) {
    timer0_millis += aMillisToAdd;
}

/*
 * disable Timer0 (millis()) overflow interrupt
 * since the loop last exactly a multiple of 1024 micros, add a few statements between disabling and enabling
 */
void disableMillisInterrupt() {
#if defined(TIMSK) && defined(TOIE)
    cbi(TIMSK, TOIE);
#elif defined(TIMSK0) && defined(TOIE0)
    cbi(TIMSK0, TOIE0); // e.g. ATmega328
#else
#error  Timer 0 overflow interrupt not disabled correctly
#endif
}

/*
 * Enable timer 0 overflow interrupt and compensate for disabled timer, if still disabled.
 */
void enableMillisInterrupt(uint16_t aMillisToAddForCompensation) {
#if defined(TIMSK) && defined(TOIE)
    if ((TIMSK & _BV(TOIE)) == 0) {
        // still disabled -> compensate
        timer0_millis += aMillisToAddForCompensation;
    }
    sbi(TIMSK, TOIE);
#elif defined(TIMSK0) && defined(TOIE0)
    if ((TIMSK0 & _BV(TOIE0)) == 0) {
        // still disabled -> compensate
        timer0_millis += aMillisToAddForCompensation;
    }
    sbi(TIMSK0, TOIE0); // e.g. ATmega328
#else
#error  Timer 0 overflow interrupt not enabled correctly
#endif
}

#endif //  defined(__AVR__)

#if ! defined(TEENSYDUINO)
void delayMilliseconds(unsigned int aMillis) {
    for (unsigned int i = 0; i < aMillis; ++i) {
        delayMicroseconds(1000);
    }
}

/*
 * returns true if aMillis were gone after the last return of true
 * Can be used as a correct non blocking replacement for delay()
 * Simple version, which can only be used at one place in code because of static variable.
 */
bool areMillisGone(unsigned int aMillis) {
    static unsigned long sLastMillis;
    if (millis() - sLastMillis >= aMillis) {
        sLastMillis = millis();
        return true;
    }
    return false;
}

bool areMillisGone(unsigned int aMillis, unsigned long *aLastMillisPtr) {
    if (millis() - *aLastMillisPtr >= aMillis) {
        *aLastMillisPtr = millis();
        return true;
    }
    return false;
}
#endif // ! defined(TEENSYDUINO)

/*
 * Function for speedTest
 * calling a function consisting of just __asm__ volatile ("nop"); gives 0 to 1 micro second
 * Use of Serial. makes it incompatible with BlueDisplay library.
 */
void speedTestWith1kCalls(Print *aSerial, void (*aFunctionUnderTest)(void)) {
    uint32_t tMillisStart = millis();
    for (uint_fast8_t i = 0; i < 100; ++i) {
        // unroll 10 times
        aFunctionUnderTest();
        aFunctionUnderTest();
        aFunctionUnderTest();
        aFunctionUnderTest();
        aFunctionUnderTest();
        aFunctionUnderTest();
        aFunctionUnderTest();
        aFunctionUnderTest();
        aFunctionUnderTest();
        aFunctionUnderTest();
    }
    uint32_t tMillisRequired = millis() - tMillisStart;
    aSerial->print(F("Function call takes "));
    if (tMillisRequired > 1000000) {
        Serial.print(tMillisRequired / 1000);
        Serial.print(",");
        Serial.print((tMillisRequired % 1000) / 100);
        Serial.print(F(" milli"));
    } else {
        Serial.print(tMillisRequired);
        Serial.print(F(" micro"));
    }
    aSerial->println(F(" seconds."));
}
