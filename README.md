# Frequency Detector Library for Arduino and ATtinys

# SUMMARY
This library analyzes a (microphone) signal and outputs the detected frequency. It simply counts zero crossings and do not use FFT.

- readSignal() is the ADC read routine, which reads 1024/512 samples and computes frequency of signal.
- doPlausi() checks if the signal is not noisy and valid. It uses the following plausibility rules:
  1. A trigger must be detected in first and last 1/8 of samples.
  2. Only 1/8 of the samples are allowed to be greater than 1.5 or less than 0.75 of the average period.
- computeDirectAndFilteredMatch() waits for n matches within a given frequency range (FrequencyMatchLow - FrequencyMatchHigh)
and also low pass filters the result for smooth transitions between the 3 match states (lower, match, greater).

## Download
The actual version can be downloaded directly from GitHub [here](https://github.com/ArminJo/Arduino-FrequencyDetector/blob/master/extras/FrequencyDetector.zip?raw=true)

# SimpleFrequencyDetector EXAMPLE
This example reads analog signal e.g. from MAX9814 Module at A1 and computes the frequency.
If frequency is in the range of 1400 to 1700 Hz, the Arduino builtin LED will light up.
It prints the detected frequency as well as plausibility errors.

SimpleFrequencyDetector on breadboard with MAX9812 Module
![SimpleFrequencyDetector on breadboard with MAX9812 Module](https://github.com/ArminJo/Arduino-FrequencyDetector/blob/master/extras/SimpleFrequencyDetector.jpg)
Youtube Demonstration of SimpleFrequencyDetector

[![Demonstration of SimpleFrequencyDetector](https://img.youtube.com/vi/tsxfSx0iY5s/0.jpg)](https://www.youtube.com/watch?v=tsxfSx0iY5s)
 
# WhistleSwitch EXAMPLE
The WhistleSwitch example analyzes a microphone signal and toggles an output pin, if the main frequency is for a specified duration in a specified range.
It works as a frequency detector for a whistle pitch which operates a mains relay. By using different pitches it is possible to control multiple relays in a single room.
If the pitch is lower than the specified frequency, the feedback LED blinks slowly, if the pitch is higher it blinks fast.
If the match holds for *MATCH_TO_LONG_MILLIS* (1000) milliseconds after switching output, the output switches again, to go back to the former state.
This can be useful if a machine generated signal (e.g. from a vacuum cleaner) matches the range.

After a timeout of TIMEOUT_RELAY_ON_SIGNAL_MINUTES (2 hours) the relay goes OFF for 1 second. 
In the next TIMEOUT_RELAY_SIGNAL_TO_OFF_MINUTES minutes you must then press the button or whistle the pitch to cancel the timeout, otherwise the relay will switch OFF.
Cancellation of timeout is acknowledged by the LED blinking 5 times for 1 second on and off. To enable the timeout, choose the dummy range 10. The setting is stored in EEPROM.


**This example is mainly created to run on an ATtiny85 @1MHz, but will work also on a plain Arduino.**

## PREDEFINED RANGES
the following pitch ranges are predefined for easy selection:
1.   1700 - 2050 Hz  -> 350 Hz
2.   1500 - 1680 Hz  -> 180 Hz
3.   1300 - 1480 Hz  -> 180 Hz
4.   1150 - 1280 Hz  -> 130 Hz
5.   1000 - 1130 Hz  -> 130 Hz
6.    900 -  990 Hz  ->  90 Hz

7.   1550 - 1900 Hz  -> 350 Hz
8.   1250 - 1530 Hz  -> 380 Hz
9.   1000 - 1230 Hz  -> 230 Hz
10.  dummy range, if chosen **enable** relay on timeout handling.
11.  dummy range, if chosen **disable** relay on timeout handling.

## SELECTING the RANGE
Selecting is started by a long press of the button.
After BUTTON_PUSH_ENTER_PROGRAM_SIMPLE_MILLIS / 1.5 seconds, the feedback LED blinks once for signaling simple selecting mode.
After BUTTON_PUSH_ENTER_PROGRAM_ADVANCED_MILLIS / 4 seconds, the feedback LED blinks twice for signaling advanced selecting mode.
After releasing the button the selecting mode is entered.

### SIMPLE SELECTING
Press the button once for range 1, twice for range 2 etc. Each button press is echoed by the feedback LED.
Waiting for PROGRAM_MODE_SIMPLE_END_DETECT_MILLIS / 1.5 seconds ends the selecting mode
and the feedback LED echoes the number of button presses recognized.
The needed duration of signal match to toggle the relay is fixed at MATCH_MILLIS_NEEDED_DEFAULT / 1.2 seconds

### ADVANCED SELECTING
Whistle the pitch you want to detect, then press the button again.
While you press the button the pitch range is measured. i.e. the minimum and maximum of the pitch you are whistling is stored.

If you press the button again before the PROGRAM_MODE_ADVANCED_END_DETECT_MILLIS / 3 sec timeout
the duration of this second press is taken as the needed duration for the signal match to toggle the relay.
Otherwise the  MATCH_MILLIS_NEEDED_DEFAULT / 1.2 seconds are taken.
After timeout of PROGRAM_MODE_TIMEOUT_MILLIS / 5 seconds the advanced selecting mode is ended
and the effective duration is echoed by the feedback LED.

## INFO / RESET
After power up or reset, the feedback LED echoes the range number. Range number 10 indicates an individual range, programmed by advanced selecting.
A reset can be performed by power off/on or by pressing the button two times each time shorter than RESET_ENTER_BUTTON_PUSH_MILLIS / 0.12 seconds
within a RESET_WAIT_TIMEOUT_MILLIS / 0.3 second interval.

# SCHEMATIC for external components of FrequencyDetector / WhistleSwitch
```
         + 5V                             _____                   o--O PIN REF
         |                             o-|_____|--o               |
         _                             |   1M     |               _
        | |                            |          |              | |
        | | 2k2                        |___|\     |              | | 1M
        |_|                            |  2| \____|              |_|
         |    ____             ____    |   | /6   |   ____   | |  |
         o---|____|-----o-----|____|---o---|/     o--|____|--| |--o--O PIN A1
         |     2k2      |      10k     |  3             10k  | |  |
        ---            |O MICROPHONE   _    LM308        10-100nF _
        --- 1 uF        |             | |                        | |
         |              |             | | 10k                    | | 1M
        ___            ___            |_|                        |_|
                                       |                          |
                                       |                          |
                                      ---                        ___
                                      ---  100 nF
                                       |
                                      ___


         + 5V               o--O PIN REF
         |                  |
         |                  _
         |                 | |
     MAX9814 MICROPHONE    | | 1M
     AMPLIFIER / MODULE    |_|
         |     ____         |
        |O ---|____|--------o--O PIN A1
         |     10k          |
         |                  _
         |                 | |
         |                 | | 1M
        ___                |_|
                            |
                            |
                           ___
```
