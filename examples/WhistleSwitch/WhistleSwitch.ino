/**
 * WhistleSwitch.cpp
 *
 * Analyzes a microphone signal and toggles an output pin, if the main frequency is for a specified duration in a specified range.
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
 *  It uses FrequencyDetector to recognize a whistle pitch which in turn operates a mains relay.
 *  By using different pitches it is possible to control multiple relays in a single room.
 *  If the pitch is lower than the specified frequency the feedback LED blinks slowly, if the pitch is higher it blinks fast.
 *  If the match holds for MATCH_TO_LONG_MILLIS (1.0) seconds after switching output, the  output switches again to go back to the former state.
 *  This can be useful if a machine generated signal (e.g. from a vacuum cleaner) matches the range.
 *
 *	RESET
 *  After power up or reset, the feedback LED echoes the range number. Range number 10 indicates an individual range, programmed by advanced programming.
 *  A reset can be performed by power off/on or by pressing the button two times each time shorter than RESET_ENTER_BUTTON_PUSH_MILLIS (0.12) seconds
 *  within a RESET_WAIT_TIMEOUT_MILLIS (0.3) second interval.
 *
 *  TIMEOUT
 *  After TIMEOUT_RELAY_ON_SIGNAL_MINUTES minutes the relay goes OFF for 1 second. In the next TIMEOUT_RELAY_SIGNAL_TO_OFF_MINUTES minutes
 *  you must then press the button or whistle the pitch to cancel the timeout, otherwise the relay will switch OFF.
 *  Cancellation of timeout is acknowledged by the LED blinking 5 times for 1 second on and off.
 *  Timeout can be switches on or off by selecting the dummy ranges 10 or 11.
 *
 *	PROGRAMMING
 *  Programming is done by a long press of the button.
 *  After BUTTON_PUSH_ENTER_PROGRAM_SIMPLE_MILLIS (1.5) seconds, the feedback LED blinks once for signaling simple programming mode.
 *  After BUTTON_PUSH_ENTER_PROGRAM_ADVANCED_MILLIS (4) seconds, the feedback LED blinks twice for signaling advanced programming mode.
 *  After releasing the button the corresponding programming mode is entered.
 *
 *  SIMPLE PROGRAMMING
 *  Press the button once for range 1, twice for range 2 etc. Each button press is echoed by the feedback LED.
 *  Waiting for PROGRAM_MODE_SIMPLE_END_DETECT_MILLIS (1.5) seconds ends the programming mode
 *  and the feedback LED echoes the number of button presses recognized.
 *  The needed duration of signal match to toggle the relay is fixed at MATCH_MILLIS_NEEDED_DEFAULT (1.2) seconds.
 *
 *	ADVANCED PROGRAMMING
 *  Whistle the pitch you want to detect, then press the button again.
 *  While you press the button the pitch range is measured. i.e. the minimum and maximum of the pitch you are whistling is stored.
 *
 *  If you press the button again before the PROGRAM_MODE_ADVANCED_END_DETECT_MILLIS (3) seconds timeout
 *  the duration of this second press is taken as the needed duration for the signal match to toggle the relay.
 *  Otherwise the  MATCH_MILLIS_NEEDED_DEFAULT (1.2) seconds are taken.
 *  After timeout of PROGRAM_MODE_TIMEOUT_MILLIS (5) seconds the advanced programming mode is ended
 *  and the effective duration is echoed by the feedback LED.
 *
 *  Since the internal oscillator is not very stable, use gaps of 20/10 Hz in order to avoid overlapping of ranges of 2 different devices
 *  PREDEFINED RANGES
 *  1   1700 - 2050 Hz  -> 350 Hz
 *  2   1500 - 1680 Hz  -> 180 Hz
 *  3   1300 - 1480 Hz  -> 180 Hz
 *  4   1150 - 1280 Hz  -> 130 Hz
 *  5   1000 - 1140 Hz  -> 140 Hz
 *  6    900 -  990 Hz  ->  90 Hz
 *
 *  7   1550 - 1900 Hz  -> 350 Hz
 *  8   1250 - 1530 Hz  -> 280 Hz
 *  9   1000 - 1230 Hz  -> 230 Hz
 *  10  dummy range, if chosen enable relay on timeout handling.
 *  11  dummy range, if chosen disable relay on timeout handling.
 *
 *  BUTTON
 *  Button state change is handled by an InterruptServiceRoutine
 *
 * FUSE VALUES for Tiny85 -> Enable BOD, run with 1 MHz
 * Low=0X62 (default) Int RC Osc. 8 MHz divided by 8. 14 Clk + 64ms startup.
 * High=0XDC BrowOut at VCC=4.3Volt
 * Extended=0XFF
 *
 * Version 3. MATCH_TO_LONG_MILLIS 1000 instead of 600
 * Version 4. Introduced MIN_NO_DROPOUT_COUNT_BEFORE_ANY_MATCH_DEFAULT to avoid short flashes on random input
 * Version 5. Programming mode feedback while button pressing + fix for advanced programming
 * Version 5.1 Adjusted highest range + Debug for Attiny
 * Version 6.0 Extracted as example and adapted to FrequencyDedector library
 * Version 6.1 3/2018 inserted gaps between ranges, added match Debug LED for 328, improved EEPROM addressing,
 *              output version number, changed MIN_NO_DROPOUT_COUNT_BEFORE_ANY_MATCH_DEFAULT from 80 to 160 ms.
 * Version 7.0 5/2018 introduced timeout for on-time duration. To enable the timeout, choose the dummy range 10. The setting is stored in EEPROM.
 *
 */

#define VERSION_EXAMPLE "7.0"

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__)
#error "Code size of this example is too large to fit in an ATtiny 25 or 45."
#endif

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
//#define INFO
#define DEBUG
#endif
#if defined(__AVR_ATtiny85__)
#define INFO
//#define DEBUG
#endif

#include <Arduino.h>
#include "FrequencyDetector.h"

/*
 * I can whistle from 550 to 1900 Hz (and do it easy from 950 - 1800)
 */
uint16_t predefinedRangesStart[] = { 1700, 1500, 1300, 1150, 1000, 900, 1550, 1250, 1000 };
uint16_t predefinedRangesEnd[] = { 2050, 1680, 1480, 1280, 1130, 990, 1900, 1530, 1230 };
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include "digitalWriteFast.h"

// ATMEL ATTINY85 - LEGACY LAYOUT
//
//                              +-\/-+
//    RESET    Ain0 (D 5) PB5  1|    |8  Vcc
//    BUTTON - Ain3 (D 3) PB3  2|    |7  PB2 (D 2) Ain1 - SIGNAL_IN
// RELAY_OUT - Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1 - DEBUG TX
//                        GND  4|    |5  PB0 (D 0) pwm0 - LED_FEEDBACK
//                              +----+
//
// ATMEL ATTINY85 -  LAYOUT_FOR_20X_AMPLIFICATION
//
//                                                     +-\/-+
//                    PCINT5/!RESET/ADC0/dW (D5) PB5  1|    |8  Vcc
//    BUTTON - PCINT3/XTAL1/CLKI/!OC1B/ADC3 (D3) PB3  2|    |7  PB2 (D2) SCK/USCK/SCL/ADC1/T0/INT0/PCINT2 - RELAY_OUT
//  SIGNAL_IN - PCINT4/XTAL2/CLKO/OC1B/ADC2 (D4) PB4  3|    |6  PB1 (D1) MISO/DO/AIN1/OC0B/OC1A/PCINT1 - LED_FEEDBACK
//                                               GND  4|    |5  PB0 (D0) MOSI/DI/SDA/AIN0/OC0A/!OC1A/AREF/PCINT0 - TX Debug output
//                                                     +----+
// ATMEL ATMEGA328 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1) SIGNAL_IN
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13) LED_FEEDBACK
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) RELAY_OUT
//      (D 7) PD7 13|    |16  PB2 (D 10) LED_HIGHER
//      (D 8) PB0 14|    |15  PB1 (D 9)  LED_MATCH
//                  +----+

/*
 *      + 5V                             _____
 *      |                             o-|_____|--o
 *      _                             |   1M     |
 *     | |                            |          |
 *     | | 4k7       ____             |  2|\     |
 *     |_| ---------|____|------------o---| \____o______o SIGNAL_IN (ADC1)
 *      | /  ____    10k      ____    |   | /6
 *      o---|____|-----o-----|____|-------|/
 *      |     4k7      |      4k7     |  3
 *     ---            |O MICROPHONE   _    LM308
 *     --- 1 uF        |             | |
 *      |              |             | | 1k
 *     ___            ___            |_|
 *                                    |
 *                                    |
 *                                   ---
 *                                   ---  100 nF
 *                                    |
 *                                   ___
 *
 *
 *
 */

#if defined(__AVR_ATtiny85__)
/*
 * Attiny85
 */
#include "TinySerialOut.h"

#define BUTTON_PIN 3

#ifdef LAYOUT_FOR_20X_AMPLIFICATION
#define RELAY_OUT 2
#define LED_FEEDBACK 1
#define DEBUG_PIN 0
#if (TX_PIN != DEBUG_PIN)
#  error "Change TX_PIN definition in TinySerialOut.h to match DEBUG_PIN."
#endif

// can only compare with low, because active from digitalReadFast() is one bit set in a byte!
#define BUTTON_PIN_ACTIVE (digitalReadFast(BUTTON_PIN) != LOW)

#define ADC_CHANNEL 7 // Differential inpit (ADC2 - ADC3) * 20
#define ADC_REFERENCE INTERNAL

#else
// Legacy layout
#define RELAY_OUT 4
#define LED_FEEDBACK 0
#define DEBUG_PIN 1
#if (TX_PIN != DEBUG_PIN)
#  error "Change TX_PIN definition in TinySerialOut.h to match DEBUG_PIN."
#endif

#define BUTTON_PIN_ACTIVE (digitalReadFast(BUTTON_PIN) == LOW)

#define ADC_CHANNEL ADC_CHANNEL_DEFAULT
#define ADC_REFERENCE DEFAULT
#endif // LAYOUT_FOR_20X_AMPLIFICATION

#define TIMING_PIN LED_FEEDBACK
#define TIMING_PORT PORTB
#endif // (__AVR_ATtiny85__)

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
/*
 * NANO
 */
#define TIMING_PIN 2
#define TIMING_PORT PORTD

#define LED_PLAUSI_FIRST  5
#define LED_PLAUSI_DISTRIBUTION  6
#define LED_SIGNAL_STRENGTH  7
#define LED_LOWER  8
#define LED_MATCH 9
#define LED_HIGHER 10

#define RELAY_OUT  11
#define LED_FEEDBACK  13
#define BUTTON_PIN 4
#define BUTTON_PIN_ACTIVE (digitalReadFast(BUTTON_PIN) == LOW)

#define ADC_CHANNEL ADC_CHANNEL_DEFAULT
#define ADC_REFERENCE DEFAULT
#endif

// plausibility for frequency in advanced programming mode
#define MAX_ACCEPTABLE_DELTA_FREQ 100 // max delta Freq for frequency change between two consecutive measurement in advanced programming mode. Used to detect dropouts.

/*
 * Timing
 */
#define MATCH_MILLIS_NEEDED_DEFAULT 1200    // number of valid period matches before relay toggle => 1200 ms
#define MATCH_TO_LONG_MILLIS        1000    // max milliseconds for match condition true, otherwise switch back to relay state before
#define RELAY_DEAD_MILLIS           1000    // min milliseconds between 2 changes of relay state -> to avoid to fast relay switching
#define BUTTON_DEBOUNCE_MILLIS      40      // must be smaller than 65 since delay micros has its limitations!
// Relay ON timeout
#define TIMEOUT_RELAY_ON_SIGNAL_MINUTES_MAX (1193 * 60) // -> 49.7 days
#define TIMEOUT_RELAY_ON_SIGNAL_MINUTES     120 // after this time, the relay is switched OFF and ON to signal timeout
#define TIMEOUT_RELAY_SIGNAL_TO_OFF_MINUTES 3   // after signaling wait this time for feedback otherwise switch the relay OFF

// LED blink timing
#define TIMING_FREQUENCY_LOWER_MILLIS     150
#define TIMING_FREQUENCY_HIGHER_MILLIS    50
#define TIMING_FREQUENCY_HIGHER_MILLIS_MINIMUM    10
/*
 * Timing for reset
 */
#define RESET_ENTER_BUTTON_PUSH_MILLIS 120  // milliseconds for first push of double push to perform reset
#define RESET_WAIT_TIMEOUT_MILLIS 300  // milliseconds to push and release button the second time in order to perform reset command
/*
 * Timing for range programming
 */
#define BUTTON_PUSH_ENTER_PROGRAM_SIMPLE_MILLIS 1500  // milliseconds to enter simple program mode
#define BUTTON_PUSH_ENTER_PROGRAM_ADVANCED_MILLIS 4000  // milliseconds to enter advanced program mode
#define PROGRAM_MODE_TIMEOUT_MILLIS 5000  // milliseconds to enter regular detect mode if in program mode and no button press happens
#define PROGRAM_MODE_SIMPLE_END_DETECT_MILLIS 1500  // milliseconds to enter regular detect mode if in simple program mode and a range has been chosen and no button press happens
#define PROGRAM_MODE_ADVANCED_END_DETECT_MILLIS 3000  // milliseconds to enter regular detect mode if in advanced program mode and a range has been entered and no button press happens
//

struct EepromParameterStruct {
    uint16_t PeriodValidNeededMillis; // ms needed for accepting match
    uint16_t FrequencyMin;
    uint16_t FrequencyMax;
};
EepromParameterStruct sPersistentParameters;
EEMEM EepromParameterStruct sPersistentParametersEEPROM;
EEMEM bool sRelayOnTimeoutIsEnabledEEPROM;

/*
 * States for main loop
 */
enum MainStateEnum {
    DETECT_FREQUENCY,
    WAIT_FOR_SECOND_PUSH_FOR_RESET,
    PROGRAM_SIMPLE,
    PROGRAM_ADVANCED_FREQUENCY_RANGE,
    PROGRAM_ADVANCED_FREQUENCY_DURATION,
    PROGRAM_WAIT_FOR_FEEDBACK_END
};

struct LedControlStruct {
    uint16_t LedBlinkMillis;
    int8_t LedBlinkCount; // >0 => blink n times, 0 => stop blinking, -1 => blink forever
    uint32_t MillisAtLastLEDChange;
} LedControl;

struct WhistleSwitchControlStruct {

    uint16_t FrequencyLast; // for advanced programming mode

    // status for main loop
    MainStateEnum MainState;
    MainStateEnum FeedbackState; // To manage feedback for long press

    uint32_t MillisAtLastRelayChange;
    //
    bool MatchJustDetected; // do only one toggle per match
    //
    bool TimeoutSignaled;  // see TIMEOUT_RELAY_ON_SIGNAL_MINUTES
    //
    bool RelayOnTimeoutIsEnabled;

    uint8_t ButtonPressCounter;

    int16_t MatchValidCount; // count for valid matches after last STATE_LED_OFF
    int16_t MatchValidNeeded; // valid matches detected needed for accepting match, i.e. for toggling relay := PeriodValidNeededMillis/timeOfReading
    uint16_t MillisNeededForValidMatch;

} WhistleSwitchControl;

struct ButtonControlStruct {
    volatile uint32_t MillisAtLastInterrupt;
    volatile bool ButtonActiveDetected;
    volatile bool ButtonReleaseJustDetected;
    volatile uint16_t MillisOfButtonPress;  // Duration of LOW state
    uint32_t MillisAtLastButtonRelease; // for reset timeout
} ButtonControl;

/*******************************************************************************************
 * Function declaration section
 *******************************************************************************************/
void handleButtonPressed();

/*
 * aLedBlinkCount = -1 -> blink forever
 */
void setFeedbackLedBlinkState(uint16_t aLedBlinkMillis, int8_t aLedBlinkCount) {
    LedControl.LedBlinkMillis = aLedBlinkMillis;
    LedControl.LedBlinkCount = aLedBlinkCount;
}

/*
 * manage LED blinking
 */
void handleLedBlinkState() {
    if (LedControl.LedBlinkCount != 0) {
        if ((millis() - LedControl.MillisAtLastLEDChange) > LedControl.LedBlinkMillis) {
            digitalToggleFast(LED_FEEDBACK);
            LedControl.MillisAtLastLEDChange = millis();
            if (digitalReadFast(LED_FEEDBACK) == LOW) {
                LedControl.LedBlinkCount--;
                if (LedControl.LedBlinkCount <= -1) {
                    LedControl.LedBlinkCount = -1;
                }
            }
        }
    }
}

void backToStateDetect() {
    WhistleSwitchControl.MainState = DETECT_FREQUENCY;
    FrequencyDetectorControl.FrequencyMatchDirect = FREQUENCY_MATCH_INVALID;
    LedControl.LedBlinkCount = 0;
}

void setMillisNeededForValidMatch(uint16_t aPeriodValidNeededMillis) {
    WhistleSwitchControl.MillisNeededForValidMatch = aPeriodValidNeededMillis;
    long tLongValue = aPeriodValidNeededMillis * 1000L;
    // subtract count from no/noisy signal to first match valid
    tLongValue -= MIN_NO_DROPOUT_COUNT_BEFORE_ANY_MATCH_DEFAULT * NUMBER_OF_SAMPLES * 52L;
#ifdef DEBUG
    // 7500 micros for debug output at 115200Baud at main loop
    WhistleSwitchControl.MatchValidNeeded = tLongValue
    / (((long) FrequencyDetectorControl.PeriodOfOneSampleMicros * NUMBER_OF_SAMPLES) + (7500 / (F_CPU / 1000000)));
#else
    WhistleSwitchControl.MatchValidNeeded = tLongValue
            / ((long) FrequencyDetectorControl.PeriodOfOneSampleMicros * NUMBER_OF_SAMPLES);
#endif
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
    Serial.print("MatchValidNeeded=");
    Serial.println(WhistleSwitchControl.MatchValidNeeded);
#else
    writeString_P(PSTR("MillisNeededForValidMatch="));
    writeUnsignedInt(WhistleSwitchControl.MillisNeededForValidMatch);
    writeValueCli('\n');
#endif
}

void eepromReadParameter() {
    eeprom_read_block((void*) &sPersistentParameters, &sPersistentParametersEEPROM, sizeof(EepromParameterStruct));
    setMillisNeededForValidMatch(sPersistentParameters.PeriodValidNeededMillis);

    FrequencyDetectorControl.FrequencyMatchLow = sPersistentParameters.FrequencyMin;
    FrequencyDetectorControl.FrequencyMatchHigh = sPersistentParameters.FrequencyMax;
    uint8_t tTimeoutFlag = eeprom_read_byte((uint8_t*) &sRelayOnTimeoutIsEnabledEEPROM);
    if (tTimeoutFlag == 0xFF) {
        WhistleSwitchControl.RelayOnTimeoutIsEnabled = false;
    } else {
        WhistleSwitchControl.RelayOnTimeoutIsEnabled = tTimeoutFlag;
    }
}

void eepromWriteParameter() {
    sPersistentParameters.FrequencyMin = FrequencyDetectorControl.FrequencyMatchLow;
    sPersistentParameters.FrequencyMax = FrequencyDetectorControl.FrequencyMatchHigh;
    sPersistentParameters.PeriodValidNeededMillis = WhistleSwitchControl.MillisNeededForValidMatch;
    eeprom_write_block((void*) &sPersistentParameters, &sPersistentParametersEEPROM, sizeof(EepromParameterStruct));
}

void eepromWriteTimeoutFlag() {
    eeprom_write_byte((uint8_t*) &sRelayOnTimeoutIsEnabledEEPROM, WhistleSwitchControl.RelayOnTimeoutIsEnabled);
#if defined(__AVR_ATtiny85__)
    writeString_P(PSTR("Timeout "));
    if (WhistleSwitchControl.RelayOnTimeoutIsEnabled) {
        writeString_P(PSTR("enabled"));
    } else {
        writeString_P(PSTR("disabled"));
    }
    writeValueCli('\n');
#else
    Serial.print("Timeout ");
    if (WhistleSwitchControl.RelayOnTimeoutIsEnabled) {
        Serial.println("enabled");
    } else {
        Serial.println("disabled");
    }
#endif

}

/*
 * echo range index
 */
void signalRangeIndexByLed() {
    // search range index
    uint8_t i;
    for (i = 0; i < (sizeof(predefinedRangesStart) / sizeof(uint16_t)); ++i) {
        if (FrequencyDetectorControl.FrequencyMatchLow == predefinedRangesStart[i]
                && FrequencyDetectorControl.FrequencyMatchHigh == predefinedRangesEnd[i]) {
            break;
        }
    }
#if defined(__AVR_ATtiny85__)
    writeString_P(PSTR("Range index="));
    writeUnsignedByte(i + 1);
    writeString_P(PSTR("\r\n"));
#else
    Serial.print("Range index=");
    Serial.println(i + 1);
#endif
    // display range index
    for (uint8_t j = 0; j <= i; ++j) {
        digitalWriteFast(LED_FEEDBACK, HIGH);
        delay(TIMING_FREQUENCY_LOWER_MILLIS * 3);
        digitalWriteFast(LED_FEEDBACK, LOW);
        delay(TIMING_FREQUENCY_LOWER_MILLIS * 3);
    }
}

/*
 * must be called as long as match is present to detect long matches, which will then reset relay to state before
 * toggles relay only once per match
 * handles timeout feedback
 * enables relay dead time TIMING_RELAY_DEAD_MILLIS
 */
void toggleRelay() {
    static bool sMatchTooLongDetected;

#if defined (INFO) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
    Serial.print("Toggle Relay. MatchJustDetected=");
    Serial.println(WhistleSwitchControl.MatchJustDetected);
#endif

    if (WhistleSwitchControl.TimeoutSignaled) {
#if defined (INFO) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
        Serial.println("Reset timeout");
#endif
        /*
         * Here we  take the next match after signaling timeout as cancellation of the timeout.
         * Signal cancellation by LED.
         */
        for (int i = 0; i < 5; ++i) {
            digitalWriteFast(LED_FEEDBACK, HIGH);
            delay(1000);
            digitalWriteFast(LED_FEEDBACK, LOW);
            delay(1000);
        }
        /*
         * Reset timeout
         */
        WhistleSwitchControl.MatchValidCount = 0; // since we do a blocking wait above
        WhistleSwitchControl.TimeoutSignaled = false;
        WhistleSwitchControl.MillisAtLastRelayChange = millis();

    } else {
        uint32_t tMillisSinceLastRelayChange = millis() - WhistleSwitchControl.MillisAtLastRelayChange;
        if (!WhistleSwitchControl.MatchJustDetected && tMillisSinceLastRelayChange > RELAY_DEAD_MILLIS) {
            /*
             * set output relay once
             */
            sMatchTooLongDetected = false;
            digitalToggleFast(RELAY_OUT);
            WhistleSwitchControl.MillisAtLastRelayChange = millis();
        } else if (!sMatchTooLongDetected && tMillisSinceLastRelayChange > MATCH_TO_LONG_MILLIS) {
            /*
             * match lasted too long, reset relay to previous state only once
             */
#if defined (INFO) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
            Serial.println("Match too long, switch to previous state.");
#endif
            digitalToggleFast(RELAY_OUT);
            sMatchTooLongDetected = true; // switch back only once
        }

    }
    WhistleSwitchControl.MatchJustDetected = true;
//    Serial.print("set MillisAtLastRelayChange=");
//    Serial.println(PfeiffschalterControl.MillisAtLastRelayChange);
}

// Example for placing code at init sections see: http://www.nongnu.org/avr-libc/user-manual/mem_sections.html
void MyInit(void) __attribute__ ((naked)) __attribute__ ((section (".init8")));
void MyInit(void) {
}
/*******************************************************************************************
 * Program code starts here
 * Setup section
 *******************************************************************************************/
void setup() {

    /* Clear flags in MCUSR */
    MCUSR = 0x00;

    /*
     * For Arduinos with other than optiboot bootloader wdt_disable() comes too late here, since after reset the watchdog is still enabled
     * and uses fastest prescaler value (approximately 15 ms)
     */ //
    wdt_disable();
#if defined(__AVR_ATtiny85__)
    initTXPin();
    useCliSeiForStrings(true);
    delay(2); // to wait for serial line to settle / stop bit
    writeString_P(PSTR("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from  " __DATE__ "\r\n"));
#else
    Serial.begin(115200);
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from  " __DATE__));
    pinMode(LED_LOWER, OUTPUT);
    pinMode(LED_MATCH, OUTPUT);
    pinMode(LED_HIGHER, OUTPUT);
    pinMode(LED_PLAUSI_FIRST, OUTPUT);
    pinMode(LED_PLAUSI_DISTRIBUTION, OUTPUT);
    pinMode(LED_SIGNAL_STRENGTH, OUTPUT);
#endif

    pinMode(LED_FEEDBACK, OUTPUT);
    pinMode(RELAY_OUT, OUTPUT);

#ifdef LAYOUT_FOR_20X_AMPLIFICATION
    pinMode(BUTTON_PIN, INPUT);
#else
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // 100kOhm to VCC
#endif

    // initial state of Pfeiffschalter
    backToStateDetect();
    ButtonControl.ButtonReleaseJustDetected = false;
    ButtonControl.ButtonActiveDetected = false;
    WhistleSwitchControl.TimeoutSignaled = false;

    /*
     * initialize FrequencyDetector with default values
     */
    setFrequencyDetectorControlDefaults();
    /*
     * Set channel, reference, sample rate and threshold for low signal detection.
     */
    setFrequencyDetectorReadingValues(ADC_CHANNEL, ADC_REFERENCE, PRESCALE_VALUE_DEFAULT, RAW_VOLTAGE_MIN_DELTA_DEFAULT);

    // get frequency parameter from eeprom
    eepromReadParameter();
    if (FrequencyDetectorControl.FrequencyMatchHigh < FrequencyDetectorControl.FrequencyMatchLow
            || FrequencyDetectorControl.FrequencyMatchLow < 300 || FrequencyDetectorControl.FrequencyMatchLow > 3000
            || FrequencyDetectorControl.FrequencyMatchHigh < 300 || FrequencyDetectorControl.FrequencyMatchHigh > 3000) {
        /*
         * eeprom not filled -> start with reasonable values
         */
        FrequencyDetectorControl.FrequencyMatchLow = predefinedRangesStart[1];
        FrequencyDetectorControl.FrequencyMatchHigh = predefinedRangesEnd[1];
        setMillisNeededForValidMatch(MATCH_MILLIS_NEEDED_DEFAULT);
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
        Serial.println("Store default values to EEPROM");
#endif
        eepromWriteParameter();
    }

#if defined(__AVR_ATtiny85__)
    writeString_P(PSTR("Frequency min="));
    writeUnsignedInt(FrequencyDetectorControl.FrequencyMatchLow);
    writeString_P(PSTR("Hz max="));
    writeUnsignedInt(FrequencyDetectorControl.FrequencyMatchHigh);
    writeString_P(PSTR("Hz\r\nTimeout "));
    if (WhistleSwitchControl.RelayOnTimeoutIsEnabled) {
        writeString_P(PSTR("enabled"));
    } else {
        writeString_P(PSTR("disabled"));
    }
    writeString_P(PSTR("\r\n"));
#else
    Serial.print("Frequency min=");
    Serial.print(FrequencyDetectorControl.FrequencyMatchLow);
    Serial.print("Hz max=");
    Serial.print(FrequencyDetectorControl.FrequencyMatchHigh);
    Serial.print("Hz\r\nTimeout ");
    if (WhistleSwitchControl.RelayOnTimeoutIsEnabled) {
        Serial.println("enabled");
    } else {
        Serial.println("disabled");
    }
#endif

    signalRangeIndexByLed();

    //initPinChangeInterrupt
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
    PCICR = 1 << PCIE2; //PCINT2 enable
    PCMSK2 = digitalPinToBitMask(BUTTON_PIN);// =0x20 - Pin 5 enable
#endif
#if defined(__AVR_ATtiny85__)
    GIMSK |= 1 << PCIE; //PCINT enable
    PCMSK = digitalPinToBitMask(BUTTON_PIN);
#endif
}

void printInfos(uint16_t aFrequency) {
#ifdef DEBUG
    static uint16_t sFrequencyFilteredPrinted;
    static uint16_t sFrequencyPrinted;
    static int16_t sTriggerLevelPrinted;
    static int16_t sAverageLevelPrinted;
    static int16_t sSignalDeltaPrinted;
    static uint8_t sMatchLowPassFilteredPrinted;

    if (sFrequencyFilteredPrinted != FrequencyDetectorControl.FrequencyFiltered || sFrequencyPrinted != aFrequency
            || sMatchLowPassFilteredPrinted != FrequencyDetectorControl.MatchLowPassFiltered
            || abs(sTriggerLevelPrinted - (int16_t)FrequencyDetectorControl.TriggerLevel) > 10
            || abs(sAverageLevelPrinted - (int16_t)FrequencyDetectorControl.AverageLevel) > 10
            || abs(sSignalDeltaPrinted - (int16_t)FrequencyDetectorControl.SignalDelta) > 10) {
        sFrequencyFilteredPrinted = FrequencyDetectorControl.FrequencyFiltered;
        sFrequencyPrinted = aFrequency;
        sMatchLowPassFilteredPrinted = FrequencyDetectorControl.MatchLowPassFiltered;
        sTriggerLevelPrinted = FrequencyDetectorControl.AverageLevel;
        sAverageLevelPrinted = FrequencyDetectorControl.TriggerLevel;
        sSignalDeltaPrinted = FrequencyDetectorControl.SignalDelta;

#if defined(__AVR_ATtiny85__)
        writeString_P(PSTR("FF="));
        writeUnsignedInt(FrequencyDetectorControl.FrequencyFiltered);
        writeString_P(PSTR("Hz F="));
        writeUnsignedInt(aFrequency);
        writeString_P(PSTR("Hz M="));
        writeUnsignedByte(FrequencyDetectorControl.MatchLowPassFiltered);
        writeString_P(PSTR(" TL="));
        writeUnsignedInt(FrequencyDetectorControl.TriggerLevel);
        writeString_P(PSTR(" AL="));
        writeUnsignedInt(FrequencyDetectorControl.AverageLevel);
        writeString_P(PSTR(" D="));
        writeUnsignedInt(FrequencyDetectorControl.SignalDelta);
        writeValueCli('\n');
#else
        Serial.print("Filtered=");
        Serial.print(FrequencyDetectorControl.FrequencyFiltered);
        Serial.print("Hz ");
        if (aFrequency <= SIGNAL_MAX_ERROR_CODE) {
            Serial.print(reinterpret_cast<const __FlashStringHelper *>(ErrorStrings[aFrequency]));
        } else {
            Serial.print("F=");
            Serial.print(aFrequency);
            Serial.print("Hz");
        }
        Serial.println();
#endif
    }
#endif
}

/************************************************************************
 * main loop
 ************************************************************************/
// noreturn saves program space!
void __attribute__((noreturn)) loop(void) {
    for (;;) {

#ifdef INFO
        static MainStateEnum sLastPrintedMainState;

        if (WhistleSwitchControl.MainState != sLastPrintedMainState) {
            sLastPrintedMainState = WhistleSwitchControl.MainState;
#if defined(__AVR_ATtiny85__)
            writeString_P(PSTR("New MainState="));
            writeUnsignedByte(WhistleSwitchControl.MainState);
            writeValueCli('\n');
#else
            Serial.print("New MainState=");
            Serial.println(WhistleSwitchControl.MainState);
#endif
        }
#endif
#if defined (TRACE) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
        Serial.print("State=");
        Serial.print(WhistleSwitchControl.MainState);
        Serial.print(" Count=");
        Serial.print(WhistleSwitchControl.LedBlinkCount);
        Serial.print(" Button=");
        Serial.print(ButtonControl.MillisOfButtonPress);
        Serial.print(" Last=");
        Serial.print(ButtonControl.MillisAtLastInterrupt);
        Serial.print(" Low=");
        Serial.println(ButtonControl.ButtonActiveDetected);
#endif
        handleLedBlinkState();

        if (WhistleSwitchControl.MainState == DETECT_FREQUENCY) {
            /*
             * Frequency processing only when button is inactive
             */
            if (BUTTON_PIN_ACTIVE) {
                handleButtonPressed();
            } else {
                // if the ISR missed the last low to high transition because of fast bouncing
                ButtonControl.ButtonActiveDetected = false;
                /*
                 * Analyze and match main loop
                 * readSignal() needs 26,6 ms for one loop at attiny85 1MHz
                 * Remaining of loop needs 260 cycles, but with debug output at 115200Baud it needs 7500 cycles.
                 */
//                BIT_SET(TIMING_PORT, TIMING_PIN);
                uint16_t tFrequency = readSignal(); // needs 26.6 millis
//                BIT_CLEAR(TIMING_PORT, TIMING_PIN);
                /*
                 * plausibility check
                 */
                tFrequency = doPlausi();

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
                digitalWriteFast(LED_SIGNAL_STRENGTH, LOW);
                digitalWriteFast(LED_LOWER, LOW);
                digitalWriteFast(LED_MATCH, LOW);
                digitalWriteFast(LED_HIGHER, LOW);
                digitalWriteFast(LED_PLAUSI_FIRST, LOW);
                digitalWriteFast(LED_PLAUSI_DISTRIBUTION, LOW);
                if (tFrequency <= SIGNAL_STRENGTH_LOW) {
                    digitalWriteFast(LED_SIGNAL_STRENGTH, HIGH);
                }
                if (tFrequency == SIGNAL_FIRST_LAST_PLAUSI_FAILED) {
                    digitalWriteFast(LED_PLAUSI_FIRST, HIGH);
                }
                if (tFrequency == SIGNAL_DISTRIBUTION_PLAUSI_FAILED) {
                    digitalWriteFast(LED_PLAUSI_DISTRIBUTION, HIGH);
                }
#endif

#ifdef DEBUG
                printInfos(tFrequency);
#endif

                /*
                 * compute match
                 */
                computeDirectAndFilteredMatch(tFrequency);
                if (FrequencyDetectorControl.FrequencyMatchDirect == FREQUENCY_MATCH_INVALID) {
                    LedControl.LedBlinkCount = 0;
                    digitalWriteFast(LED_FEEDBACK, LOW);
                }
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
                if (FrequencyDetectorControl.FrequencyMatchDirect == FREQUENCY_MATCH_LOWER) {
                    digitalWriteFast(LED_LOWER, HIGH);
                } else if (FrequencyDetectorControl.FrequencyMatchDirect == FREQUENCY_MATCH_HIGHER) {
                    digitalWriteFast(LED_HIGHER, HIGH);
                } else if (FrequencyDetectorControl.FrequencyMatchDirect == FREQUENCY_MATCH) {
                    digitalWriteFast(LED_MATCH, HIGH);
                }
#endif

                /*
                 * process match state
                 * set led blinking and count valid matches until toggling relay
                 */
                if (FrequencyDetectorControl.FrequencyMatchFiltered == FREQUENCY_MATCH_INVALID) {
                    WhistleSwitchControl.MatchValidCount = 0;
                } else if (FrequencyDetectorControl.FrequencyMatchFiltered == FREQUENCY_MATCH_HIGHER) {
                    // HIGHER -> set blink frequency according to gap between real and maximum-match pitch
                    int16_t tLedBlinkMillis = TIMING_FREQUENCY_HIGHER_MILLIS
                            - ((FrequencyDetectorControl.FrequencyFiltered - FrequencyDetectorControl.FrequencyMatchHigh) / 16);
                    if (tLedBlinkMillis < TIMING_FREQUENCY_HIGHER_MILLIS_MINIMUM) {
                        tLedBlinkMillis = TIMING_FREQUENCY_HIGHER_MILLIS_MINIMUM;
                    }

#if defined (DEBUG) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
                    Serial.print("tLedBlinkMillis=");
                    Serial.println(tLedBlinkMillis);
#endif
                    setFeedbackLedBlinkState(tLedBlinkMillis, -1);
                    WhistleSwitchControl.MatchValidCount--;
                } else if (FrequencyDetectorControl.FrequencyMatchFiltered == FREQUENCY_MATCH_LOWER) {
                    // LOWER -> set blink frequency according to gap between real and minimal-match pitch
                    uint16_t tLedBlinkMillis = TIMING_FREQUENCY_LOWER_MILLIS
                            + ((FrequencyDetectorControl.FrequencyMatchLow - FrequencyDetectorControl.FrequencyFiltered) / 2);
#if defined (DEBUG) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))

                    Serial.print("tLedBlinkMillis=");
                    Serial.println(tLedBlinkMillis);
#endif
                    setFeedbackLedBlinkState(tLedBlinkMillis, -1);
                    WhistleSwitchControl.MatchValidCount--;
                } else {
                    /*
                     * successful match here
                     */
                    LedControl.LedBlinkCount = 0;
                    WhistleSwitchControl.MatchValidCount++;
                    digitalWriteFast(LED_FEEDBACK, HIGH);
                }

                /*
                 * housekeeping
                 */
                if (WhistleSwitchControl.MatchValidCount < 0) {
                    WhistleSwitchControl.MatchValidCount = 0;
                }
                if (WhistleSwitchControl.MatchValidCount >= WhistleSwitchControl.MatchValidNeeded) {
                    /*
                     * valid match here  - "MatchJustDetected = true" is be set in toggleRelay()
                     */
                    toggleRelay();
                } else {
                    WhistleSwitchControl.MatchJustDetected = false;
                }

                /*
                 * Button handling
                 */
                if (ButtonControl.ButtonReleaseJustDetected) {
                    ButtonControl.ButtonReleaseJustDetected = false;
                    /*
                     * set next state / toggle relay
                     */
                    if (ButtonControl.MillisOfButtonPress < RESET_ENTER_BUTTON_PUSH_MILLIS) {
                        // wait for next push for reset
                        WhistleSwitchControl.MainState = WAIT_FOR_SECOND_PUSH_FOR_RESET;
                        ButtonControl.MillisAtLastButtonRelease = millis();
                    } else if (WhistleSwitchControl.FeedbackState == DETECT_FREQUENCY) {
                        /*
                         * and action ...
                         */
                        toggleRelay();
                    } else {
                        // set programming state
                        WhistleSwitchControl.MainState = WhistleSwitchControl.FeedbackState;
                        WhistleSwitchControl.FeedbackState = DETECT_FREQUENCY;
                        WhistleSwitchControl.ButtonPressCounter = 0;
                    }

#if defined (INFO)
#if (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
                    Serial.print("MillisOfButtonPress=");
                    Serial.print(ButtonControl.MillisOfButtonPress);
                    Serial.print(" State=");
                    Serial.println(WhistleSwitchControl.MainState);
#else
                    writeString_P(PSTR("BMillis="));
                    writeUnsignedInt(ButtonControl.MillisOfButtonPress);
                    writeString_P(PSTR(" State="));
                    writeUnsignedByte(WhistleSwitchControl.MainState);
                    write1Start8Data1StopNoParity('\n');
#endif
#endif
                }

                if (WhistleSwitchControl.RelayOnTimeoutIsEnabled) {
                    /*
                     * Check for Relay to be ON for more than TIMEOUT_RELAY_ON_SIGNAL_MINUTES minutes
                     */
                    bool tRelayState = digitalReadFast(RELAY_OUT);
                    if (tRelayState) {
                        uint32_t tMillisSinceLastRelayChange = millis() - WhistleSwitchControl.MillisAtLastRelayChange;
                        if (tMillisSinceLastRelayChange > TIMEOUT_RELAY_ON_SIGNAL_MINUTES * 60000L) {
                            if (tMillisSinceLastRelayChange
                                    > (TIMEOUT_RELAY_ON_SIGNAL_MINUTES + TIMEOUT_RELAY_SIGNAL_TO_OFF_MINUTES) * 60000L) {
#if defined (INFO) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
                                Serial.println("Timeout detected. Switch off.");
                                Serial.print("MillisAtLastRelayChange=");
                                Serial.println(WhistleSwitchControl.MillisAtLastRelayChange);
#endif
                                /*
                                 * No cancellation signal received -> switch off relay
                                 */
                                digitalWriteFast(RELAY_OUT, LOW);
                                WhistleSwitchControl.TimeoutSignaled = false;
                            } else if (!WhistleSwitchControl.TimeoutSignaled) {
#if defined (INFO) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
                                Serial.println("Timeout detected. Signal it.");
#endif
                                digitalWriteFast(RELAY_OUT, LOW);
                                delay(1500);
                                digitalWriteFast(RELAY_OUT, HIGH);
                                WhistleSwitchControl.TimeoutSignaled = true;
                            }
                        }
                    }
                }
            }

            /*
             * Frequency detect state END
             */
        } else if (WhistleSwitchControl.MainState == WAIT_FOR_SECOND_PUSH_FOR_RESET) {
            /*************************************************************************************
             * reset -> feedback state
             *************************************************************************************/
            // IF second push happens before timeout then perform reset, otherwise just switch relay
            if (millis() - ButtonControl.MillisAtLastButtonRelease > RESET_WAIT_TIMEOUT_MILLIS) {
                // Reset state and toggle relay
                WhistleSwitchControl.MainState = DETECT_FREQUENCY;
                WhistleSwitchControl.MatchJustDetected = false;
                toggleRelay();

#if defined (DEBUG) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
                Serial.println("Timeout for reset");
#endif
            } else if (ButtonControl.ButtonReleaseJustDetected) {
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
#if defined (INFO)
                Serial.println("Simulate reset");
#endif
                WhistleSwitchControl.MainState = DETECT_FREQUENCY;
                digitalWriteFast(RELAY_OUT, LOW);
                signalRangeIndexByLed();
                ButtonControl.ButtonReleaseJustDetected = false;
#else
                // second push happened before timeout -> perform reset (this does not work with arduino bootloader)
                wdt_enable(WDTO_500MS);
                while (1) {
                };
#endif
            }
        } else if (WhistleSwitchControl.MainState == PROGRAM_WAIT_FOR_FEEDBACK_END) {
            /*************************************************************************************
             * Wait for feedback to end
             *************************************************************************************/
            if (LedControl.LedBlinkCount <= 0) {
                backToStateDetect();
            }
        } else {
            /*********************************************************************
             * PROGRAMMING starts here
             *********************************************************************/
            // get new values, since the ISR can have changed it
            uint32_t tLastButtonInterruptMillis = ButtonControl.MillisAtLastInterrupt;
            uint32_t tMillis = millis();
            // check for timeout
            if (tMillis - tLastButtonInterruptMillis > PROGRAM_MODE_TIMEOUT_MILLIS) {
                ButtonControl.MillisAtLastInterrupt = tMillis; // avoid endless timeouts ;-)
                setFeedbackLedBlinkState(TIMING_FREQUENCY_LOWER_MILLIS, 1);
                WhistleSwitchControl.MainState = PROGRAM_WAIT_FOR_FEEDBACK_END;

#if defined (INFO) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
                Serial.println("timeout");
#endif
            } else if (WhistleSwitchControl.MainState == PROGRAM_SIMPLE) {
                /*
                 * PROGRAM_SIMPLE
                 */
                if (ButtonControl.ButtonReleaseJustDetected) {
                    // increment range count
                    ButtonControl.ButtonReleaseJustDetected = false;
                    WhistleSwitchControl.ButtonPressCounter++;
                    if (WhistleSwitchControl.ButtonPressCounter > ((sizeof(predefinedRangesStart) / sizeof(uint16_t)) + 2)) {
                        WhistleSwitchControl.ButtonPressCounter = ((sizeof(predefinedRangesStart) / sizeof(uint16_t)) + 2);
                    }

#ifdef INFO
#if defined(__AVR_ATtiny85__)
                    writeString_P(PSTR("Count="));
                    writeUnsignedByte(WhistleSwitchControl.ButtonPressCounter);
                    writeValueCli('\n');

#else
                    Serial.print("Count=");
                    Serial.println(WhistleSwitchControl.ButtonPressCounter);
#endif
#endif
                    // echo button
                    setFeedbackLedBlinkState(TIMING_FREQUENCY_LOWER_MILLIS * 3, 1);
                } else if ((WhistleSwitchControl.ButtonPressCounter > 0)
                        && (tMillis - tLastButtonInterruptMillis > PROGRAM_MODE_SIMPLE_END_DETECT_MILLIS)) {
                    /*
                     * Timeout detected -> set new range
                     */
                    if (WhistleSwitchControl.ButtonPressCounter > (sizeof(predefinedRangesStart) / sizeof(uint16_t))) {
                        /*
                         * Set relay on timeout
                         */
                        if (WhistleSwitchControl.ButtonPressCounter == (sizeof(predefinedRangesStart) / sizeof(uint16_t)) + 1) {
                            WhistleSwitchControl.RelayOnTimeoutIsEnabled = true;
                        } else {
                            WhistleSwitchControl.RelayOnTimeoutIsEnabled = false;
                        }
                        eepromWriteTimeoutFlag();
                    } else {
                        FrequencyDetectorControl.FrequencyMatchLow = predefinedRangesStart[WhistleSwitchControl.ButtonPressCounter
                                - 1];
                        FrequencyDetectorControl.FrequencyMatchHigh = predefinedRangesEnd[WhistleSwitchControl.ButtonPressCounter
                                - 1];
                        setMillisNeededForValidMatch(MATCH_MILLIS_NEEDED_DEFAULT);
                        eepromWriteParameter();
                    }
                    // echo recognized parameter
                    setFeedbackLedBlinkState(TIMING_FREQUENCY_LOWER_MILLIS * 3, WhistleSwitchControl.ButtonPressCounter);
                    WhistleSwitchControl.MainState = PROGRAM_WAIT_FOR_FEEDBACK_END;
                }

            } else if (WhistleSwitchControl.MainState == PROGRAM_ADVANCED_FREQUENCY_RANGE) {
                /*
                 * PROGRAM_ADVANCED_FREQUENCY_RANGE
                 */
                if (BUTTON_PIN_ACTIVE) {
                    //get range values
                    digitalWriteFast(LED_FEEDBACK, HIGH);
                    WhistleSwitchControl.ButtonPressCounter = 1;
                    readSignal();                         // 27.6 millis
                    uint16_t tFrequency = doPlausi();
                    if (tFrequency > SIGNAL_MAX_ERROR_CODE) {
                        // if frequency changes to fast, we assume a dropout
                        if (((WhistleSwitchControl.FrequencyLast + MAX_ACCEPTABLE_DELTA_FREQ) - tFrequency)
                                < (2 * MAX_ACCEPTABLE_DELTA_FREQ) || FrequencyDetectorControl.FrequencyMatchHigh == 0) {
                            WhistleSwitchControl.FrequencyLast = tFrequency;
                            if (tFrequency > FrequencyDetectorControl.FrequencyMatchHigh) {
                                FrequencyDetectorControl.FrequencyMatchHigh = tFrequency;
                            }
                            if (tFrequency < FrequencyDetectorControl.FrequencyMatchLow) {
                                FrequencyDetectorControl.FrequencyMatchLow = tFrequency;
                            }
                        }
                    }

#if defined (INFO) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
                    Serial.print(" last=");
                    Serial.print(WhistleSwitchControl.FrequencyLast);
                    Serial.print(" act=");
                    Serial.print(tFrequency);
                    Serial.print(" min=");
                    Serial.print(FrequencyDetectorControl.FrequencyMatchLow);
                    Serial.print("Hz max=");
                    Serial.print(FrequencyDetectorControl.FrequencyMatchHigh);
                    Serial.println("Hz");

#endif
                } else if (WhistleSwitchControl.ButtonPressCounter > 0) {
                    digitalWriteFast(LED_FEEDBACK, LOW);
                    WhistleSwitchControl.MainState = PROGRAM_ADVANCED_FREQUENCY_DURATION;
                    ButtonControl.ButtonReleaseJustDetected = false; // reset flag
                }
            } else if (WhistleSwitchControl.MainState == PROGRAM_ADVANCED_FREQUENCY_DURATION) {
                // end detection (timeout handling) if a range has been chosen
                if (tMillis - tLastButtonInterruptMillis > PROGRAM_MODE_ADVANCED_END_DETECT_MILLIS) {
// set duration to default because it is not specified by additional button press
                    setMillisNeededForValidMatch(MATCH_MILLIS_NEEDED_DEFAULT);
// write parameter for range to eeprom here since values are valid and programming ends here
                    eepromWriteParameter();
// echo duration
                    setFeedbackLedBlinkState(WhistleSwitchControl.MillisNeededForValidMatch, 1);
                    WhistleSwitchControl.MainState = PROGRAM_WAIT_FOR_FEEDBACK_END;
                } else if (ButtonControl.ButtonReleaseJustDetected) {
                    /*
                     * PROGRAM_ADVANCED_FREQUENCY_DURATION
                     */
                    ButtonControl.ButtonReleaseJustDetected = false;
                    setMillisNeededForValidMatch(ButtonControl.MillisOfButtonPress);
// write parameter with duration to eeprom
                    eepromWriteParameter();

#if defined (INFO) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
                    Serial.print("MillisOfButtonPress=");
                    Serial.print(ButtonControl.MillisOfButtonPress);
                    Serial.print(" duration=");
                    Serial.println(WhistleSwitchControl.MillisNeededForValidMatch);
#endif
// echo duration
                    setFeedbackLedBlinkState(WhistleSwitchControl.MillisNeededForValidMatch, 1);
                    // forces delay until led on
                    LedControl.MillisAtLastLEDChange = tMillis;

                    WhistleSwitchControl.MainState = PROGRAM_WAIT_FOR_FEEDBACK_END;
                }
            }
        }
    }
    // for(;;)
}

/*
 *  check for long press timings in order to signal and prepare state changes
 */
void handleButtonPressed() {
    uint32_t tMillisSinceLastInterrupt = millis() - ButtonControl.MillisAtLastInterrupt;
    // debouncing is needed here too!
    if (tMillisSinceLastInterrupt >= BUTTON_DEBOUNCE_MILLIS) {
        ButtonControl.MillisOfButtonPress = tMillisSinceLastInterrupt;
        /*
         * prepare for state change on ButtonReleaseJustDetected
         * signal programming state while button is still pressed and not after release
         */
        if (ButtonControl.MillisOfButtonPress > BUTTON_PUSH_ENTER_PROGRAM_SIMPLE_MILLIS) {
            if (WhistleSwitchControl.FeedbackState == DETECT_FREQUENCY) {
#if defined (INFO) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
                Serial.println("Prepare for simple programming state");
#endif
                /*
                 * Prepare for PROGRAM_SIMPLE but do not change mainState here!
                 * Changing state is done in main loop upon button release.
                 */
                WhistleSwitchControl.FeedbackState = PROGRAM_SIMPLE;
                setFeedbackLedBlinkState(TIMING_FREQUENCY_LOWER_MILLIS * 3, 1);
            } else if (ButtonControl.MillisOfButtonPress > BUTTON_PUSH_ENTER_PROGRAM_ADVANCED_MILLIS) {
                if (WhistleSwitchControl.FeedbackState == PROGRAM_SIMPLE) {
#if defined (INFO) && (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
                    Serial.println("Prepare for advanced programming state");
#endif
                    // prepare for PROGRAM_ADVANCED_FREQUENCY_RANGE
                    WhistleSwitchControl.FeedbackState = PROGRAM_ADVANCED_FREQUENCY_RANGE;
                    FrequencyDetectorControl.FrequencyMatchLow = 1000;
                    FrequencyDetectorControl.FrequencyMatchHigh = 0;
                    setFeedbackLedBlinkState(TIMING_FREQUENCY_LOWER_MILLIS * 3, 2);
                }
            }
        }
    }
}

/*
 * Button change handler
 * Problem: we are not sure that the level we read, is the level which triggers the interrupt, therefore do debouncing here
 * compute active period duration
 */
//
#if defined(__AVR_ATtiny85__)
ISR(PCINT0_vect) {
#else
    ISR(PCINT2_vect) {
#endif
    uint32_t tMillis = millis();
    uint32_t tMillisSinceLastInterrupt = tMillis - ButtonControl.MillisAtLastInterrupt;
    ButtonControl.MillisAtLastInterrupt = tMillis;
    // use delayMicroseconds() since we are in an interrupt service routine and delay(), using the timer, is not working
    delayMicroseconds(BUTTON_DEBOUNCE_MILLIS * 1000U);
    if (BUTTON_PIN_ACTIVE) {
        ButtonControl.ButtonActiveDetected = true;
    } else if (ButtonControl.ButtonActiveDetected) {
// Button release here
        ButtonControl.ButtonActiveDetected = false;
        ButtonControl.MillisOfButtonPress = tMillisSinceLastInterrupt;
        ButtonControl.ButtonReleaseJustDetected = true;
    }
}

