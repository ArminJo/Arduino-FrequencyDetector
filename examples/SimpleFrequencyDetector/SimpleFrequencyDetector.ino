/*
 * SimpleFrequencyDetector.cpp
 *
 * SimpleFrequencyDetector reads an analog signal e.g. from a MAX9814 Module at A1 and computes the frequency.
 * If frequency is in the range of 1400 to 1700 Hz, the Arduino built in LED will light up.
 *
 *
 *  Copyright (C) 2014-2025  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-FrequencyDetector https://github.com/ArminJo/Arduino-FrequencyDetector.
 *
 *  Arduino-FrequencyDetector is free software: you can redistribute it and/or modify
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

/*
 *      + 5 V / 3.3 V         o--O PIN REF
 *      |                     |
 *      |                     _
 *      |                    | |
 * MAX4466 / 9814 MICROPHONE | | 1 M
 *  AMPLIFIER / MODULE       |_|
 *      |                     |
 *     |O ---------||---------o--O PIN A1
 *      |         10 nF       |
 *      |                     _
 *      |                    | |
 *      |                    | | 1 M
 *     ___                   |_|
 *                            |
 *                            |
 *                           ___
 *
 */

#include <Arduino.h>

/*
 * Pin definitions for real time status info with LEDs
 */
#define LED_PIN_NO_TRIGGER                  5
#define LED_PIN_SIGNAL_STRENGTH_LOW         6
#define LED_PIN_FREQUENCY_TOO_LOW_OR_HIGH   7
#define LED_PIN_PLAUSI_DISTRIBUTION_FAILED  8

//#define INFO
#if ! defined(LED_BUILTIN) && defined(ARDUINO_AVR_DIGISPARK)
#  if defined(DIGISTUMPCORE)
#define LED_BUILTIN PB1
#  else
#define LED_BUILTIN PIN_PB1
#  endif
#endif

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#  if defined(DIGISTUMPCORE)
#define TX_PIN PB2 // (package pin 7 on Tiny85) - can use one of PB0 to PB4 (+PB5) here
#  endif
#include "ATtinySerialOut.hpp" // Available as Arduino library "ATtinySerialOut"
#endif

//#define PRINT_INPUT_SIGNAL_TO_PLOTTER     // If enabled, store SIGNAL_PLOTTER_BUFFER_SIZE input samples for printing to Arduino Plotter
#include "FrequencyDetector.hpp"

//#define PRINT_RESULTS_TO_SERIAL_PLOTTER   // If enabled, this example program prints generated output values to Arduino Serial Plotter (Ctrl-Shift-L)
#if defined(PRINT_INPUT_SIGNAL_TO_PLOTTER) && defined(PRINT_RESULTS_TO_SERIAL_PLOTTER)
#error Please define only one of PRINT_INPUT_SIGNAL_TO_PLOTTER or PRINT_RESULTS_TO_SERIAL_PLOTTER
#endif

#if defined(INFO)
#include "AVRUtils.h" // for printRAMInfo()
#endif

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // initialize the digital real time LED status info pins as output
    pinMode(LED_PIN_NO_TRIGGER, OUTPUT);
    pinMode(LED_PIN_SIGNAL_STRENGTH_LOW, OUTPUT);
    pinMode(LED_PIN_FREQUENCY_TOO_LOW_OR_HIGH, OUTPUT);
    pinMode(LED_PIN_PLAUSI_DISTRIBUTION_FAILED, OUTPUT);

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
#if !defined(PRINT_INPUT_SIGNAL_TO_PLOTTER)
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_FREQUENCY_DETECTOR));
    Serial.println(
            F(
                    "LED for no trigger at pin " STR(LED_PIN_NO_TRIGGER) ", for signal strength too low  at pin " STR(LED_PIN_SIGNAL_STRENGTH_LOW) ", for frequency too low or too high at pin " STR(LED_PIN_FREQUENCY_TOO_LOW_OR_HIGH) ", for distribution plausi fail at pin " STR(LED_PIN_PLAUSI_DISTRIBUTION_FAILED)));

#endif

    /*
     * initialize default values for dropout counts for frequency detector.
     */
    setFrequencyDropoutDefaults();

    /*
     * Set channel, reference, sample rate and threshold for low signal detection.
     * Set reference to 1.1Volt for AC coupled signal.
     * This is equivalent to an additional signal amplification of around 4.
     */
    setFrequencyDetectorReadingValues(ADC_CHANNEL_DEFAULT, INTERNAL, PRESCALE_VALUE_DEFAULT, RAW_VOLTAGE_MIN_DELTA_DEFAULT);

    // set my Frequency range
    setFrequencyDetectorMatchValues(1400, 1700);
#if !defined(PRINT_INPUT_SIGNAL_TO_PLOTTER)
    printFrequencyMatchValues(&Serial);
#endif
#if defined(INFO)
    // do not use printCurrentAvailableStackSize(&Serial) because we may use the ATtinySerialOut as Serial
    Serial.print(F("Currently available Stack[bytes]="));
    Serial.println(getCurrentAvailableStackSize());
#endif
}

void loop() {
    /*
     * Read samples and compute and output frequency and do plausi.
     */
    uint16_t tFrequency = readSignal();
    tFrequency = doEqualDistributionPlausi();
    computeDirectAndFilteredMatch(tFrequency);
    //    printPeriodLengthArray(&Serial);

    /*
     * Show (error) status on LED's
     */
    digitalWrite(LED_PIN_SIGNAL_STRENGTH_LOW, LOW);
    digitalWrite(LED_PIN_FREQUENCY_TOO_LOW_OR_HIGH, LOW);
    digitalWrite(LED_PIN_PLAUSI_DISTRIBUTION_FAILED, LOW);
    digitalWrite(LED_PIN_NO_TRIGGER, LOW);

    if (tFrequency == SIGNAL_STRENGTH_LOW) {
        digitalWrite(LED_PIN_SIGNAL_STRENGTH_LOW, HIGH);
    }
    if (tFrequency == SIGNAL_FREQUENCY_TOO_LOW || tFrequency == SIGNAL_FREQUENCY_TOO_HIGH) {
        digitalWrite(LED_PIN_FREQUENCY_TOO_LOW_OR_HIGH, HIGH);
    }
    if (tFrequency == SIGNAL_DISTRIBUTION_PLAUSI_FAILED) {
        digitalWrite(LED_PIN_PLAUSI_DISTRIBUTION_FAILED, HIGH);
    }
    if (tFrequency == SIGNAL_NO_TRIGGER) {
        digitalWrite(LED_PIN_NO_TRIGGER, HIGH);
    }

#if defined(PRINT_RESULTS_TO_SERIAL_PLOTTER)
    /*
     *  Print computed values to Arduino Serial Plotter
     */
    printDataForArduinoPlotter(&Serial);
#endif

#if defined(PRINT_INPUT_SIGNAL_TO_PLOTTER)
    printInputSignalValuesForArduinoPlotter(&Serial);
#endif

    //reset match indicator led
    digitalWrite(LED_BUILTIN, LOW);

    if (tFrequency > SIGNAL_MAX_ERROR_CODE) {
        Serial.print(tFrequency);
        Serial.print(F("Hz "));
        /*
         * No signal errors here -> compute match
         */
        if (FrequencyDetectorControl.FrequencyMatchDirect == FREQUENCY_MATCH) {
            // signal match
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            Serial.print(F("no "));
        }
        Serial.println(F("match"));

#if !defined(PRINT_RESULTS_TO_SERIAL_PLOTTER) && !defined(PRINT_INPUT_SIGNAL_TO_PLOTTER)
    } else {
        // incompatible with Serial Plotter
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(ErrorStrings[tFrequency]));
#endif
    }
}
