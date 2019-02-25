/*
 * FrequencyDetector.h
 *
 * Analyzes a microphone signal and outputs the detected frequency.
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

#ifndef FREQUENCYDETECTOR_H_
#define FREQUENCYDETECTOR_H_

// Propagate debug level
#ifdef TRACE
#define DEBUG
#endif
#ifdef DEBUG
#define INFO
#endif
#ifdef INFO
#define WARN
#endif
#ifdef WARN
#define ERROR
#endif

/*
 * Global settings which are needed at compile time
 */
/*
 * Number of samples used for detecting the frequency of the signal.
 * 1024 -> 53.248 milliseconds / 18.78 Hz at 16 MHz clock with prescaler 64 and 13 cycles/sample (=> 52usec/sample | 19230 Hz sample rate)
 *  For frequency below 400Hz it might be good to change PRESCALE_VALUE_DEFAULT from PRESCALE64 to PRESCALE128.
 *  1024 -> 106.496 milliseconds / 9.39 Hz at 16 MHz clock with prescaler 128 and 13 cycles/sample (=> 104usec/sample | 9615 Hz sample rate)
 *
 *  512 -> 26.624 milliseconds / 37.56 Hz at  1 MHz clock with prescaler  4 and 13 cycles/sample (=> 52usec/sample | 19230 Hz sample rate)
 *
 */
#if defined(__AVR_ATtiny85__)
#define NUMBER_OF_SAMPLES 512
#else
#define NUMBER_OF_SAMPLES 1024
#endif

/*
 * Defaults for reading
 */
#define ADC_CHANNEL_DEFAULT 1 // Channel ADC1 (PB2 on ATtiny85)
#define RAW_VOLTAGE_MIN_DELTA_DEFAULT 0x40 // 1/16 of max amplitude for minimum signal strength

/*
 * Defaults for plausibility
 */
#define MIN_SAMPLES_PER_PERIOD 8   // => Max frequency is 2403 Hz at 52usec/sample
// Fixed values for plausibility
#define LEADING_TRAILING_TRIGGER_MARGIN (NUMBER_OF_SAMPLES / 8) // Margin for doPlausi() where at least one trigger (eg. TriggerFirstPosition) must be detected
#define SIZE_OF_PERIOD_LENGTH_ARRAY_FOR_PLAUSI (NUMBER_OF_SAMPLES / MIN_SAMPLES_PER_PERIOD)

/*
 * Defaults for match
 */
#define FREQUENCY_MIN_DEFAULT 1000
#define FREQUENCY_MAX_DEFAULT 2000

#if NUMBER_OF_SAMPLES == 512
// 6 -> 160 milliseconds for 512 samples
#define MAX_DROPOUT_COUNT_BEFORE_NO_FILTERED_MATCH_DEFAULT 8   // 212ms
#define MIN_NO_DROPOUT_COUNT_BEFORE_ANY_MATCH_DEFAULT 12        // - to avoid short flashes at random signal input
#else
// 3 -> 160 milliseconds for 1024 samples at 52usec/sample
#define MAX_DROPOUT_COUNT_BEFORE_NO_FILTERED_MATCH_DEFAULT 3 // number of allowed error (FrequencyActual <= SIGNAL_MAX_ERROR_CODE) conditions, before match = FREQUENCY_MATCH_INVALID
#define MIN_NO_DROPOUT_COUNT_BEFORE_ANY_MATCH_DEFAULT 6 // number of needed valid readings (FrequencyActual > SIGNAL_MAX_ERROR_CODE) before any (lower, match, higher) match - to avoid short flashes at random signal input
#endif

// sample time values for Prescaler for 16 MHz 4(13*0,25=3,25us), 8(6,5us), 16(13us), 32(26us), 64(52us), 128(104us)
#define ADC_PRESCALE2    1
#define ADC_PRESCALE4    2
#define ADC_PRESCALE8    3
#define ADC_PRESCALE16   4
#define ADC_PRESCALE32   5
#define ADC_PRESCALE64   6
#define ADC_PRESCALE128  7

/*
 * Default timing for reading -> 19,23 kHz sample rate
 * Formula is F_CPU / (PrescaleFactor * 13)
 * For frequency below 400Hz it might be good to increase PRESCALE_VALUE_DEFAULT from PRESCALE64 to PRESCALE128.
 * For frequencies above 3kHz it might be good to decrease PRESCALE_VALUE_DEFAULT from PRESCALE64 to PRESCALE32 or even lower.
 */
#if F_CPU == 16000000L
#define PRESCALE_VALUE_DEFAULT ADC_PRESCALE64 // 52 microseconds per ADC sample at 16 Mhz Clock => 19,23kHz sample rate
#elif F_CPU == 8000000L
#define PRESCALE_VALUE_DEFAULT ADC_PRESCALE32 // 52 microseconds per ADC sample at 8 Mhz Clock => 19,23kHz sample rate
#elif F_CPU == 1000000L
#define PRESCALE_VALUE_DEFAULT ADC_PRESCALE4 // 52 microseconds per ADC sample at 1 Mhz Clock => 19,23kHz sample rate
#endif

/*
 * storage for millis value to enable compensation for interrupt disable at signal acquisition etc.
 */
#if ( defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) )
#define timer0_millis millis_timer_millis // The ATTinyCore libraries use other variable name in wiring.c
#endif
extern volatile unsigned long timer0_millis;

// FrequencyActual error values
#define SIGNAL_NO_TRIGGER 0
#define SIGNAL_STRENGTH_LOW 1
// You get this error code if no trigger occurs in the first or last 128 samples because signal is noisy or or only a burst
#define SIGNAL_FIRST_LAST_PLAUSI_FAILED 2
// You get this error code if more than 1/8 of the samples are greater than 1.5 or less than 0.75 of the average period
#define SIGNAL_DISTRIBUTION_PLAUSI_FAILED 3
#define SIGNAL_MAX_ERROR_CODE 3 // the highest error value

const char ErrorString_0[] PROGMEM = "No trigger";
const char ErrorString_1[] PROGMEM = "Signal low";
const char ErrorString_2[] PROGMEM = "No signal zero crossing at start or end of sample";
const char ErrorString_3[] PROGMEM = "Periods between signal zero crossing during the sample are too different";
extern const char *ErrorStrings[SIGNAL_MAX_ERROR_CODE + 1];

const char ErrorStringShort_2[] PROGMEM = "No 0 xing at start or end";
const char ErrorStringShort_3[] PROGMEM = "Periods too different";
extern const char *ErrorStringsShort[SIGNAL_MAX_ERROR_CODE + 1];
// Result values for Match*
enum MatchStateEnum {
    FREQUENCY_MATCH_INVALID /*Errors have happened*/, FREQUENCY_MATCH_LOWER, FREQUENCY_MATCH, FREQUENCY_MATCH_HIGHER
};

/*
 * Values for MatchLowPassFiltered
 * Valid values are filtered values from 50 to 150
 */
#define FILTER_VALUE_MAX        200
#define FILTER_VALUE_MIN        0
#define FILTER_VALUE_MIDDLE     ((FILTER_VALUE_MAX + FILTER_VALUE_MIN)/2)
#define FILTER_VALUE_THRESHOLD  (FILTER_VALUE_MIDDLE/2)
#define FILTER_VALUE_MATCH      FILTER_VALUE_MIDDLE
#define FILTER_VALUE_MATCH_HIGHER_THRESHOLD     (FILTER_VALUE_MAX - FILTER_VALUE_THRESHOLD)
#define FILTER_VALUE_MATCH_LOWER_THRESHOLD      (FILTER_VALUE_MIN + FILTER_VALUE_THRESHOLD)

struct FrequencyDetectorControlStruct {

    /**********************************************
     * All values are used or set by readSignal()
     *********************************************/
    // INPUT
    /*
     * 3 Values set by setFrequencyDetectorReadingPrescaleValue()
     */
    uint8_t ADCPrescalerValue;
    uint16_t FrequencyOfOneSample;    // to compute the frequency from the number of samples of one signal wave
    uint16_t PeriodOfOneSampleMicros; // to compute the matches needed from the number of loops

    /*
     * Value set by setFrequencyDetectorReadingValues()
     * Minimum signal strength value to produce valid output and do new trigger level computation. Otherwise return SIGNAL_STRENGTH_LOW
     */
    uint16_t RawVoltageMinDelta; // Threshold for minimum SignalDelta of raw ADC value for valid signal strength. 0x40=312mV at 5V and 68.75mY at 1.1V, 0x20=156/34,37 mVolt

    // INTERNALLY
    /*
     * internally computed values for automatic trigger level adjustment
     */
    uint16_t TriggerLevel; // = MinValue + ((MaxValue - MinValue)/2)
    uint16_t TriggerLevelLower; // = TriggerLevel - (tDelta / 8) - for Hysteresis

    // OUTPUT
    /*
     * Values of sampled signal input
     */
    uint16_t SignalDelta; // MaxValue - MinValue
    uint16_t AverageLevel;  // = SumOfSampleValues / NumberOfSamples

    /*
     * Values computed by readSignal() to be used by doPlausi()
     */
    uint16_t FrequencyActual;   // Frequency in Hz set by readSignal() or "error code"  SIGNAL_... set by doPlausi()
    uint8_t PeriodCountActual; // Actual count of periods in all samples - !!! cannot be greater than SIZE_OF_PERIOD_LEGTH_ARRAY_FOR_PLAUSI - 1)!!!
    uint8_t PeriodLength[SIZE_OF_PERIOD_LENGTH_ARRAY_FOR_PLAUSI]; // Array of period length of the signal for plausi
    uint16_t TriggerFirstPosition; // position of first detection of a trigger in all samples
    uint16_t TriggerLastPosition;  // position of last detection of a trigger in all samples

    /**************************************************
     * 9 Parameters for computeDirectAndFilteredMatch()
     *************************************************/
    // INPUT
    uint16_t FrequencyMatchLow;   // Thresholds for matching
    uint16_t FrequencyMatchHigh;

    uint8_t MaxMatchDropoutCount; // number of allowed error (FrequencyActual <= SIGNAL_MAX_ERROR_CODE) conditions, before match = FREQUENCY_MATCH_INVALID
    uint8_t MinMatchNODropoutCount; // number of needed valid readings (FrequencyActual > SIGNAL_MAX_ERROR_CODE) before any (lower, match, higher) match - to avoid short flashes at random signal input
    // INTERNALLY
    uint8_t MatchDropoutCount;      // actual dropout count. If value falls below MaxMatchDropoutCount, filtered match is valid.

    // OUTPUT
    uint16_t FrequencyFiltered;   // Low pass filter value for frequency, e.g. to compute stable difference to target frequency.

    MatchStateEnum FrequencyMatchDirect; // Result of match: 0 to 3, FREQUENCY_MATCH_INVALID, FREQUENCY_MATCH_LOWER, FREQUENCY_MATCH, FREQUENCY_MATCH_HIGHER
    MatchStateEnum FrequencyMatchFiltered; // same range asFrequencyMatchDirect. Match state processed by low pass filter
    // INTERNALLY
    uint8_t MatchLowPassFiltered;    // internal value 0 to FILTER_VALUE_MAX/200. Low pass filter value for computing FrequencyMatchFiltered
};

extern FrequencyDetectorControlStruct FrequencyDetectorControl;

void setFrequencyDetectorControlDefaults();
void setFrequencyDetectorReadingDefaults();
void setFrequencyDetectorReadingValues(uint8_t aADCChannel, uint8_t aADCReference, uint8_t aADCPrescalerValue,
        uint16_t aRawVoltageMinDelta);
void setFrequencyDetectorReadingPrescaleValue(uint8_t aADCPrescalerValue);
void setFrequencyDetectorMatchValues(uint16_t aFrequencyMin, uint16_t aFrequencyMax);
void setFrequencyDetectorDropoutValues(uint8_t aMinMatchNODropoutCount, uint8_t aMaxMatchDropoutCount);

uint16_t readSignal();
uint16_t doPlausi();
void computeDirectAndFilteredMatch(uint16_t aFrequency);

#endif /* FREQUENCYDETECTOR_H_ */
