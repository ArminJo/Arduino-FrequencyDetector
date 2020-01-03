/**
 * FrequencyDetector.cpp
 *
 * Analyzes a microphone signal and outputs the detected frequency. It simply counts zero crossings and do not use FFT.
 * The ADC sample data is NOT stored in RAM, only the period lengths are stored in the PeriodLength[] array,
 * which is a byte array and has the size of NUMBER_OF_SAMPLES / 8.
 *
 * The timer 0 interrupt, which counts the milliseconds, is disabled during reading and enabled afterwards!
 * The value of millis() is adjusted after reading.
 * The alternative to disabling the interrupt is getting partially invalid results!
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
 *  FEATURES;
 *  readSignal() is the ADC read routine, which reads 1024/512 samples and computes frequency of signal.
 *
 *  doPlausi() checks if the signal is not noisy and valid. It uses the following plausibility rules:
 *      1. A trigger must be detected in first and last 1/8 of samples
 *      2. Only 1/8 of the samples are allowed to be greater than 1.5 or less than 0.75 of the average period
 *
 *  computeDirectAndFilteredMatch() wait for n matches within a given frequency range (FrequencyMatchLow - FrequencyMatchHigh)
 *  and also low pass filters the result for smooth transitions between the 3 match states (lower, match, greater)
 *
 */

#include <Arduino.h>

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#include "ATtinySerialOut.h"
#endif

//#define INFO
//#define DEBUG
//#define TRACE

#include "FrequencyDetector.h"

#define maximumAllowableCountOf(aPeriodCountTotal) (aPeriodCountTotal / 8)

FrequencyDetectorControlStruct FrequencyDetectorControl;

const char *ErrorStrings[] = { ErrorString_0, ErrorString_1, ErrorString_2, ErrorString_3 };
const char *ErrorStringsShort[] = { ErrorString_0, ErrorString_1, ErrorStringShort_2, ErrorStringShort_3 };

// Union to speed up the combination of low and high bytes to a word
// it is not optimal since the compiler still generates 2 unnecessary moves
// but using  -- value = (high << 8) | low -- gives 5 unnecessary instructions
union Myword {
    struct {
        uint8_t LowByte;
        uint8_t HighByte;
    } byte;
    uint16_t Word;
    uint8_t * BytePointer;
};

// definitions from <wiring_private.h>
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/****************************************************************
 * Code starts here
 ***************************************************************/
/*
 * aADCChannel can be 0 to 7 or A0 to A7
 * aADCReference can be DEFAULT (VCC) or INTERNAL (1.1 volt)
 * aADCPrescalerValue can be one of PRESCALE4, PRESCALE8, PRESCALE32, PRESCALE64 or PRESCALE128
 * aFrequencyOfOneSampleTimes100 depends on value of aADCPrescalerValue
 * Formula is
 */
void setFrequencyDetectorReadingValues(uint8_t aADCChannel, const uint8_t aADCReference, uint8_t aADCPrescalerValue,
        uint16_t aRawVoltageMinDelta) {

    // Setup ADC, setMUX + Reference
    if (aADCChannel >= 14) {
        aADCChannel -= 14; // allow for channel or pin numbers
    }

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = (aADCReference << REFS2) | aADCChannel;
#else
    ADMUX = (aADCReference << REFS0) | aADCChannel;
#endif

    setFrequencyDetectorReadingPrescaleValue(aADCPrescalerValue);
    FrequencyDetectorControl.RawVoltageMinDelta = aRawVoltageMinDelta;
}

void setFrequencyDetectorReadingPrescaleValue(uint8_t aADCPrescalerValue) {
    FrequencyDetectorControl.ADCPrescalerValue = aADCPrescalerValue;
    //Formula is F_CPU / (PrescaleFactor * 13)
    FrequencyDetectorControl.PeriodOfOneSampleMicros = ((1 << aADCPrescalerValue) * 13) / (F_CPU / 1000000L);
    FrequencyDetectorControl.PeriodOfOneReadingMillis = ((FrequencyDetectorControl.PeriodOfOneSampleMicros
            * (uint32_t) NUMBER_OF_SAMPLES) + CLOCKS_FOR_READING_NO_LOOP) / 1000;
    uint32_t tFrequencyOfOneSample = 1000000L / FrequencyDetectorControl.PeriodOfOneSampleMicros;
    FrequencyDetectorControl.FrequencyOfOneSample = tFrequencyOfOneSample;

#ifdef INFO
    Serial.print(F("SamplePeriod="));
    Serial.print(FrequencyDetectorControl.PeriodOfOneSampleMicros);
    Serial.println(F("us"));
#endif
}

void setFrequencyDetectorMatchValues(uint16_t aFrequencyMin, uint16_t aFrequencyMax) {
    FrequencyDetectorControl.FrequencyMatchLow = aFrequencyMin;
    FrequencyDetectorControl.FrequencyMatchHigh = aFrequencyMax;
}

void setFrequencyDetectorDropoutCounts(uint8_t aMinMatchNODropoutCount, uint8_t aMaxMatchDropoutCount) {
    FrequencyDetectorControl.MinMatchNODropoutCount = aMinMatchNODropoutCount;
    FrequencyDetectorControl.MaxMatchDropoutCount = aMaxMatchDropoutCount;
}

/*
 * Computes MinMatchNODropoutCount and MaxMatchDropoutCount.
 * If program size matters, use setFrequencyDetectorDropoutCounts() instead or set them directly.
 * @return true if values set, false if PeriodOfOneReadingMillis == 0
 */
bool setFrequencyDetectorDropoutTimes(uint16_t aMinMatchNODropoutMillis, uint16_t aMaxMatchDropoutMillis) {
    bool tRetval = false;
    if (FrequencyDetectorControl.PeriodOfOneReadingMillis != 0) {
        FrequencyDetectorControl.MinMatchNODropoutCount = aMinMatchNODropoutMillis
                / FrequencyDetectorControl.PeriodOfOneReadingMillis;
        FrequencyDetectorControl.MaxMatchDropoutCount = aMaxMatchDropoutMillis / FrequencyDetectorControl.PeriodOfOneReadingMillis;
        tRetval = true;
    } else{
#ifdef INFO
    Serial.println(F("Error. Values not set! Must call setFrequencyDetectorReadingPrescaleValue() before!"));
#endif
    }
#ifdef INFO
    Serial.print(F("MinMatchNODropoutCount="));
    Serial.print(FrequencyDetectorControl.MinMatchNODropoutCount);
    Serial.print(F(" MaxMatchDropoutCount="));
    Serial.println(FrequencyDetectorControl.MaxMatchDropoutCount);
#endif
    return tRetval;
}

void setFrequencyDetectorControlDefaults() {
    setFrequencyDetectorMatchValues(FREQUENCY_MIN_DEFAULT, FREQUENCY_MAX_DEFAULT);
    setFrequencyDetectorDropoutCounts(MIN_NO_DROPOUT_COUNT_BEFORE_ANY_MATCH_DEFAULT,
    MAX_DROPOUT_COUNT_BEFORE_NO_FILTERED_MATCH_DEFAULT);
}

void setFrequencyDetectorReadingDefaults() {
    // for DC coupled signal
    setFrequencyDetectorReadingValues(ADC_CHANNEL_DEFAULT, DEFAULT, PRESCALE_VALUE_DEFAULT, RAW_VOLTAGE_MIN_DELTA_DEFAULT);
    // set reference to 1.1 volt for AC coupled signal - is equivalent to an additional signal amplification of around 4
    //setFrequencyDetectorReadingValues(ADC_CHANNEL_DEFAULT, INTERNAL1V1, PRESCALE_VALUE_DEFAULT, RAW_VOLTAGE_MIN_DELTA_DEFAULT);
}

/*
 * ADC read routine reads NUMBER_OF_SAMPLES (1024/512) samples and computes:
 * - FrequencyDetectorControl.FrequencyActual - Frequency of signal
 * or error value SIGNAL_STRENGTH_LOW if signal is too weak
 *
 * Sets the next trigger levels for hysteresis of 1/8 peak to peak value:
 * - FrequencyDetectorControl.TriggerLevelLower - for next reading
 * - FrequencyDetectorControl.TriggerLevelUpper - for next reading
 *
 * Sets values for plausibility check:
 * - FrequencyDetectorControl.PeriodLength[]
 * - FrequencyDetectorControl.PeriodCountActual
 * - FrequencyDetectorControl.TriggerFirstPosition
 * - FrequencyDetectorControl.TriggerLastPosition
 * Triggering value for next reading, trigger hysteresis is 1/8 peak to peak value
 *
 * !!! Timer0 interrupt, which counts the milliseconds is disabled during reading and enabled afterwards!!!
 * The alternative of using disable interrupt is getting wrong results!!!
 * The value of millis() is adjusted manually after reading.
 */
uint16_t readSignal() {
    Myword tUValue;
    /*
     * disable Timer0 (millis()) overflow interrupt
     */
#if defined(ARDUINO_AVR_DIGISPARK)
    // Digispark uses timer1 for millis()
        cbi(TIMSK, TOIE1);
#else
#  if defined(TIMSK) && defined(TOIE0)
    cbi(TIMSK, TOIE0);
#  elif defined(TIMSK0) && defined(TOIE0)
    cbi(TIMSK0, TOIE0);
#  else
#error  Timer 0 overflow interrupt not disabled correctly
#  endif
#endif // defined(ARDUINO_AVR_DIGISPARK)

//  ADCSRB = 0; // free running mode  - is default
    ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | FrequencyDetectorControl.ADCPrescalerValue);

    bool tTriggerSearchStart = true;
    bool tSignalTriggerFound = false;
    uint16_t tPeriodCountPosition = 0;

    // Initialize max and min
    uint16_t tValueMax = 0;
    uint16_t tValueMin = 1024;
    uint32_t tSumOfSampleValues = 0;

    uint8_t tPeriodCountActual = 0;

    /*
     * Read 512/1024 samples
     */
    for (uint16_t i = 0; i < NUMBER_OF_SAMPLES; i++) {
        // loop takes 22 cycles and we have 52 cycles @1MHz between each conversion :-)
        /*
         * wait for free running conversion to finish.
         * Do not wait for ADSC here, since ADSC is only low for 1 ADC Clock cycle on free running conversion.
         */
        loop_until_bit_is_set(ADCSRA, ADIF);
        // Get value
        tUValue.byte.LowByte = ADCL;
        tUValue.byte.HighByte = ADCH;
        ADCSRA |= (1 << ADIF); // clear bit to recognize next conversion has finished

        /*
         * Detect trigger
         */
        if (tTriggerSearchStart) {
            // rising slope - wait for value below 1. threshold
            if (tUValue.Word < FrequencyDetectorControl.TriggerLevelLower) {
                tTriggerSearchStart = false;
            }
        } else {
            // rising slope - wait for value to rise above 2. threshold
            if (tUValue.Word > FrequencyDetectorControl.TriggerLevel) {
                //Trigger found but skip first (incomplete period)
                if (tSignalTriggerFound) {
                    FrequencyDetectorControl.PeriodLength[tPeriodCountActual] = i - tPeriodCountPosition;
                    if (tPeriodCountActual < (sizeof(FrequencyDetectorControl.PeriodLength) - 1)) {
                        tPeriodCountActual++;
                    }
                    tPeriodCountPosition = i;
                } else {
                    FrequencyDetectorControl.TriggerFirstPosition = i;
                    tPeriodCountPosition = i;
                }
                tSignalTriggerFound = true;
                tTriggerSearchStart = true;
            }
        }

        // Get average for external statistics
        tSumOfSampleValues += tUValue.Word;

        // get max and min for automatic triggering
        if (tUValue.Word > tValueMax) {
            tValueMax = tUValue.Word;
        } else if (tUValue.Word < tValueMin) {
            tValueMin = tUValue.Word;
        }
    }

    ADCSRA &= ~(1 << ADATE); // Disable ADC auto-triggering

    FrequencyDetectorControl.AverageLevel = tSumOfSampleValues / NUMBER_OF_SAMPLES;
    FrequencyDetectorControl.TriggerLastPosition = tPeriodCountPosition;
    FrequencyDetectorControl.PeriodCountActual = tPeriodCountActual;

    uint16_t tDelta = tValueMax - tValueMin;
    FrequencyDetectorControl.SignalDelta = tDelta;
    // middle between min and max
    uint16_t tTriggerValue = tValueMin + (tDelta / 2);
    FrequencyDetectorControl.TriggerLevel = tTriggerValue;

    /*
     * Enable timer 0 overflow interrupt and compensate for disabled timer, if still disabled.
     * We need 625 microseconds for other computations @1MHz.
     */
#if defined(ARDUINO_AVR_DIGISPARK)
    // Digispark uses timer1 for millis()
    if ((TIMSK & _BV(TOIE1)) == 0) {
        // still disabled -> compensate
        timer0_millis += FrequencyDetectorControl.PeriodOfOneReadingMillis;
    }
    sbi(TIMSK, TOIE1);
#else // defined(ARDUINO_AVR_DIGISPARK)
#  if defined(TIMSK) && defined(TOIE0)
    if ((TIMSK & _BV(TOIE0)) == 0) {
        // still disabled -> compensate
        timer0_millis += FrequencyDetectorControl.PeriodOfOneReadingMillis;
    }
    sbi(TIMSK, TOIE0);
#  elif defined(TIMSK0) && defined(TOIE0)
    if ((TIMSK0 & _BV(TOIE0)) == 0) {
        // still disabled -> compensate
        timer0_millis += FrequencyDetectorControl.PeriodOfOneReadingMillis;
    }
    sbi(TIMSK0, TOIE0);
#  else
#error  Timer 0 overflow interrupt not enabled correctly
#  endif
#endif // defined(ARDUINO_AVR_DIGISPARK)

    /*
     * check for signal strength
     */
    if (tDelta < FrequencyDetectorControl.RawVoltageMinDelta) {
        // don't compute new TriggerHysteresis value because signal is too low!!!!
        FrequencyDetectorControl.FrequencyActual = SIGNAL_STRENGTH_LOW;
    } else {

        // set hysteresis
        uint8_t TriggerHysteresis = tDelta / 8;
        FrequencyDetectorControl.TriggerLevelLower = tTriggerValue - TriggerHysteresis;

        if (tPeriodCountActual == 0) {
            FrequencyDetectorControl.FrequencyActual = SIGNAL_NO_TRIGGER; // Frequency too low
        } else {

            /*
             * Must use long intermediate value to avoid 16 Bit overflow
             * (FrequencyDetectorControl.FrequencyOfOneSample / Number of samples) => frequency for one period in number of samples
             */
            FrequencyDetectorControl.FrequencyActual = ((long) tPeriodCountActual * FrequencyDetectorControl.FrequencyOfOneSample)
                    / (FrequencyDetectorControl.TriggerLastPosition - FrequencyDetectorControl.TriggerFirstPosition);

#ifdef DEBUG
            Serial.print(F("Delta U="));
            Serial.print(tDelta);
            Serial.print(F(" TriggerValue="));
            Serial.print(tTriggerValue);
            Serial.print(F(" PeriodCount="));
            Serial.print(FrequencyDetectorControl.PeriodCountActual);
            Serial.print(F(" Samples="));
            Serial.print(FrequencyDetectorControl.TriggerLastPosition - FrequencyDetectorControl.TriggerFirstPosition);
            Serial.print(F(" Freq="));
            Serial.println(FrequencyDetectorControl.FrequencyActual);
#endif
        }
    }
    return FrequencyDetectorControl.FrequencyActual;
}

/** Overwrite FrequencyDetectorControl.FrequencyActual with these error values if plausibility check fails:
 *      SIGNAL_FIRST_PLAUSI_FAILED 2
 *      SIGNAL_DISTRIBUTION_PLAUSI_FAILED 3
 * Used plausibility rules are:
 * 1. A trigger must be detected in first and last 1/8 of samples
 * 2. Only 1/8 of the samples are allowed to be greater than 1.5 or less than 0.75 of the average period
 * @return the (changed) FrequencyDetectorControl.FrequencyActual
 */
uint16_t doPlausi() {
    if (FrequencyDetectorControl.FrequencyActual > SIGNAL_STRENGTH_LOW) {
        /*
         * plausibility check
         */
        if (FrequencyDetectorControl.TriggerFirstPosition >= LEADING_TRAILING_TRIGGER_MARGIN
                || FrequencyDetectorControl.TriggerLastPosition <= NUMBER_OF_SAMPLES - LEADING_TRAILING_TRIGGER_MARGIN) {
            FrequencyDetectorControl.FrequencyActual = SIGNAL_FIRST_LAST_PLAUSI_FAILED;

        } else {
            uint8_t tPeriodCountActual = FrequencyDetectorControl.PeriodCountActual;
            /*
             * check if not more than 1/8 of periods are out of range - less than 0.75 or more than 1.5
             */
            // average can be between 8 an 32
            uint8_t tAveragePeriod = (FrequencyDetectorControl.TriggerLastPosition - FrequencyDetectorControl.TriggerFirstPosition)
                    / tPeriodCountActual;
            uint8_t tPeriodMax = tAveragePeriod + (tAveragePeriod / 2);
            uint8_t tPeriodMin = tAveragePeriod - (tAveragePeriod / 4);
            uint8_t tErrorCount = 0;
#ifdef TRACE
            Serial.print(tAveragePeriod);
            Serial.print("  ");
#endif
            for (uint8_t i = 0; i < tPeriodCountActual; ++i) {
#ifdef TRACE
                Serial.print(FrequencyDetectorControl.PeriodLength[i]);
                Serial.print(",");
#endif
                if (FrequencyDetectorControl.PeriodLength[i] > tPeriodMax
                        || FrequencyDetectorControl.PeriodLength[i] < tPeriodMin) {
                    tErrorCount++;
                }
            }
            if (tErrorCount > maximumAllowableCountOf(tPeriodCountActual)) {
                FrequencyDetectorControl.FrequencyActual = SIGNAL_DISTRIBUTION_PLAUSI_FAILED;
            }
#ifdef TRACE
            Serial.print(tErrorCount);
            Serial.print(F("  #="));
            Serial.print(tPeriodCountActual);
            Serial.print(F("  F="));
            Serial.print(FrequencyDetectorControl.FrequencyActual);
            Serial.println(F("Hz"));

#endif
        }
    }
    return FrequencyDetectorControl.FrequencyActual;
}

/**
 * simple low-pass filter over 15 values
 */
uint16_t LowPassFilterWith16Values(uint16_t aFilteredValue, uint16_t aValue) {
    return (((aFilteredValue * 15) + aValue + (1 << 3)) >> 4); // (tValue+8)/16 (+8 to avoid rounding errors)
}

/**
 * handles dropouts / no signal
 * dropout count is between 0 and MaxMatchDropoutCount and on latter the match will be reset.
 * determine direct match state - FrequencyDetectorControl.FrequencyMatch
 * computes low-pass filtered match state FrequencyMatchFiltered and frequency FrequencyMatchFiltered
 */
void computeDirectAndFilteredMatch(uint16_t aFrequency) {

    if (aFrequency <= SIGNAL_MAX_ERROR_CODE) {
        /*
         *  dropout / no signal handling
         */
        FrequencyDetectorControl.FrequencyMatchDirect = FREQUENCY_MATCH_INVALID;
        FrequencyDetectorControl.MatchDropoutCount++;
        if (FrequencyDetectorControl.MatchDropoutCount > FrequencyDetectorControl.MaxMatchDropoutCount) {
            FrequencyDetectorControl.FrequencyMatchFiltered = FREQUENCY_MATCH_INVALID;

            /*
             * clip value at MaxMatchDropoutCount + MinMatchNODropoutCount, so at least MinMatchNODropoutCount matches must happen
             * to set FrequencyMatchFiltered not to FREQUENCY_MATCH_INVALID
             */
            if (FrequencyDetectorControl.MatchDropoutCount
                    >= FrequencyDetectorControl.MaxMatchDropoutCount + FrequencyDetectorControl.MinMatchNODropoutCount) {
                FrequencyDetectorControl.MatchDropoutCount = FrequencyDetectorControl.MaxMatchDropoutCount
                        + FrequencyDetectorControl.MinMatchNODropoutCount;
            }
        }
    } else {
        /*
         * Valid signal
         * decrement dropout count until 0
         */
        if (FrequencyDetectorControl.MatchDropoutCount > 0) {
            FrequencyDetectorControl.MatchDropoutCount--;
        }

        /*
         * Determine direct match state and tNewFilterValue for low pass filter
         */
        uint8_t tNewFilterValue;
        if (aFrequency < FrequencyDetectorControl.FrequencyMatchLow) {
            // Frequency too low
            FrequencyDetectorControl.FrequencyMatchDirect = FREQUENCY_MATCH_LOWER;
            tNewFilterValue = FILTER_VALUE_MIN;
        } else if (aFrequency > FrequencyDetectorControl.FrequencyMatchHigh) {
            // Frequency too high
            FrequencyDetectorControl.FrequencyMatchDirect = FREQUENCY_MATCH_HIGHER;
            tNewFilterValue = FILTER_VALUE_MAX;
        } else {
            // Frequency matches
            FrequencyDetectorControl.FrequencyMatchDirect = FREQUENCY_MATCH;
            tNewFilterValue = FILTER_VALUE_MATCH;
        }

        /*
         * Low pass filter for smooth transitions between the 3 match states (lower, match, higher)
         */
        if (FrequencyDetectorControl.MatchDropoutCount == FrequencyDetectorControl.MaxMatchDropoutCount) {
            // init filter at first time when signal is valid
            FrequencyDetectorControl.MatchLowPassFiltered = tNewFilterValue;
            FrequencyDetectorControl.FrequencyFiltered = aFrequency;
        } else if (FrequencyDetectorControl.MatchDropoutCount < FrequencyDetectorControl.MaxMatchDropoutCount) {
            /*
             * Low pass filter the frequency and the match
             */
            FrequencyDetectorControl.FrequencyFiltered = LowPassFilterWith16Values(FrequencyDetectorControl.FrequencyFiltered,
                    aFrequency);
            FrequencyDetectorControl.MatchLowPassFiltered = LowPassFilterWith16Values(FrequencyDetectorControl.MatchLowPassFiltered,
                    tNewFilterValue);
        }

        /*
         * determine new low pass filtered match state
         */
        if (FrequencyDetectorControl.MatchDropoutCount > FrequencyDetectorControl.MaxMatchDropoutCount) {
            // too much dropouts
            FrequencyDetectorControl.FrequencyMatchFiltered = FREQUENCY_MATCH_INVALID;
        } else {
            if (FrequencyDetectorControl.MatchLowPassFiltered > FILTER_VALUE_MATCH_HIGHER_THRESHOLD) {
                FrequencyDetectorControl.FrequencyMatchFiltered = FREQUENCY_MATCH_HIGHER;
            } else if (FrequencyDetectorControl.MatchLowPassFiltered < FILTER_VALUE_MATCH_LOWER_THRESHOLD) {
                FrequencyDetectorControl.FrequencyMatchFiltered = FREQUENCY_MATCH_LOWER;
            } else {
                FrequencyDetectorControl.FrequencyMatchFiltered = FREQUENCY_MATCH;
            }
        }
    }
}

