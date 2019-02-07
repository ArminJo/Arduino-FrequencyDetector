/*
 * SimpleFrequencyDetector.cpp
 *
 * SimpleFrequencyDetector reads an analog signal e.g. from a MAX9814 Module at A1 and computes the frequency.
 * If frequency is in the range of 1400 to 1700 Hz, the Arduino built in LED will light up.
 *
 *
 *  Copyright (C) 2014  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

/*
 *      + 5V / 3.3V           o--O PIN REF
 *      |                     |
 *      |                     _
 *      |                    | |
 * MAX4466 / 9814 MICROPHONE | | 1M
 *  AMPLIFIER / MODULE       |_|
 *      |                     |
 *     |O ---------||---------o--O PIN A1
 *      |         10nF        |
 *      |                     _
 *      |                    | |
 *      |                    | | 1M
 *     ___                   |_|
 *                            |
 *                            |
 *                           ___
 *
 */

#include <Arduino.h>
#include "FrequencyDetector.h"

#define VERSION_EXAMPLE "1.0"

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    /*
     * initialize FrequencyDetector
     */
    setFrequencyDetectorControlDefaults();
    /*
     * Set channel, reference, sample rate and threshold for low signal detection.
     * Set reference to 1.1Volt for AC coupled signal.
     * This is equivalent to an additional signal amplification of around 4.
     */
    setFrequencyDetectorReadingValues(ADC_CHANNEL_DEFAULT, INTERNAL, PRESCALE_VALUE_DEFAULT, RAW_VOLTAGE_MIN_DELTA_DEFAULT);

    // set my Frequency range
    setFrequencyDetectorMatchValues(1400, 1700);
}

void loop() {
    /*
     * Read samples and compute and output frequency
     */
    uint16_t tFrequency = readSignal();

    //reset match indicator led
    digitalWrite(LED_BUILTIN, LOW);

    if (tFrequency > SIGNAL_MAX_ERROR_CODE) {
        // print value for Arduino Serial Plotter
        Serial.println(tFrequency);

        /*
         * Do plausibility check and print plausibility errors
         */
        tFrequency = doPlausi();
        if (tFrequency <= SIGNAL_MAX_ERROR_CODE) {
            Serial.println(reinterpret_cast<const __FlashStringHelper *>(ErrorStrings[tFrequency]));
        } else {
            /*
             * No signal errors here -> compute match
             */
            computeDirectAndFilteredMatch(tFrequency);
            if (FrequencyDetectorControl.FrequencyMatchDirect == FREQUENCY_MATCH) {
                // signal match
                digitalWrite(LED_BUILTIN, HIGH);
            }
        }
    }
}
