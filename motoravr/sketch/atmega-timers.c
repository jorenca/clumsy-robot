//      atmega-timers.c
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

#include "atmega-timers.h"

// patch
#define TIMSK1 TIMSK

void (*_t1_func)();

void timer1(uint8_t prescaler, uint16_t ticks, void (*f)()) {
	TIMSK1 &= ~(_BV(OCIE1A));
	_t1_func = f;
	OCR1A = ticks;
	TCCR1A = 0;
	TCCR1B = prescaler | _BV(WGM12);
	TCNT1 = 0;
	TIMSK1 |= _BV(OCIE1A);
}

void timer1_stop() {
	TCCR1B = 0;
}

#ifdef ENABLE_TIMER1
ISR(TIMER1_COMPA_vect) {
	_t1_func();
}
#endif
