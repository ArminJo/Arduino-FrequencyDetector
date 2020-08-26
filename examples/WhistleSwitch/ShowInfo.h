/*
 * ShowInfo.h
 *
 *  Copyright (C) 2018-2019  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-Utils https://github.com/ArminJo/Arduino-Utils.
 *
 *  Arduino-Utils is free software: you can redistribute it and/or modify
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

#ifndef SHOWINFO_H_
#define SHOWINFO_H_

#include <Arduino.h>

// Based on https://playground.arduino.cc/Main/ShowInfo

float GetTemp(void);

// Helper function for free ram.
//   With use of http://playground.arduino.cc/Code/AvailableMemory
//
int freeRam(void);


// Helper function for sketch size.
// The sketch size is runtime calculated.
// From user "Coding Badly" in his post:
//   http://arduino.cc/forum/index.php/topic,115870.msg872309.html#msg872309
// Changed into unsigned long for code size larger than 64kB.
//
// This function returns the sketch size
// for a size between 0 and 32k. If the code
// size is larger (for example with an Arduino Mega),
// the return value is not valid.
//
unsigned long sketchSize(void);

void Information(void);

#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny167__) || defined(__AVR_ATtiny87__)
void printBODLevel(uint8_t aHighFuseBits);
void printBODLevel();
void printFuses(void);
void printMCUSR(uint8_t aMCUSRContent);
void printBODSFlagExistence();
#endif //  defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny167__) || defined(__AVR_ATtiny87__)

#if !defined (__AVR_ATmega32U4__)
#  if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny167__) || defined(__AVR_ATtiny87__)
/*
 * Short version using printHex and saving Flash
 */
void TimerCommonRegisterDump(void);
void Timer0RegisterDump(void);
void Timer1RegisterDump(void);
void TimerRegisterDump(void);

#    if ! defined(ARDUINO_AVR_DIGISPARKPRO)
void ADCChannelDump(void);
#    endif

#  else // defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny167__) || defined(__AVR_ATtiny87__)
void Timer0RegisterDump(void);
#    if defined(TCCR1A)
void Timer1RegisterDump(void) ;
#    endif

#    if defined(TCCR2A)
void Timer2RegisterDump(void);
#    endif

void TimerRegisterDump(void);
#  endif
#endif // !defined (__AVR_ATmega32U4__)

#endif /* SHOWINFO_H_ */

#pragma once
