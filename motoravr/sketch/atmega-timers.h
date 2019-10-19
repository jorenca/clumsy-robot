//      atmega-timers.h
//
//      Copyright 2011 Javier Valencia <javiervalencia80@gmail.com>
//
//      This program is free software; you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation; either version 2 of the License, or
//      (at your option) any later version.
//
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//
//      You should have received a copy of the GNU General Public License
//      along with this program; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
//      MA 02110-1301, USA.
//
//

#ifndef atmega_timers_h
#define atmega_timers_h

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

// Comment/delete those lines to disable interrupt definition for that timer
// so you can define your own ISR functions without conflicts.
#define ENABLE_TIMER1

#define TIMER1_PRESCALER_NONE 0
#define TIMER1_PRESCALER_1 1
#define TIMER1_PRESCALER_8 2
#define TIMER1_PRESCALER_64 3
#define TIMER1_PRESCALER_256 4
#define TIMER1_PRESCALER_1024 5

void timer1(uint8_t prescaler, uint16_t ticks, void (*f)());
void timer1_stop();

#ifdef __cplusplus
}
#endif

#endif
