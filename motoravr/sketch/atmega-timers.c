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

void (*_t1_func)();
void (*_t2_func)();

void timer1(uint8_t prescaler, uint16_t ticks, void (*f)()) {
	TIMSK &= ~(_BV(OCIE1A));
	_t1_func = f;
	OCR1A = ticks;
	TCCR1A = 0;
	TCCR1B = prescaler | _BV(WGM12);
	TCNT1 = 0;
	TIMSK |= _BV(OCIE1A);
}

void timer1_stop() {
	TCCR1B = 0;
}

#ifdef ENABLE_TIMER1
ISR(TIMER1_COMPA_vect) {
	_t1_func();
}
#endif

void timer2(uint8_t prescaler, uint8_t ticks, void (*f)()) {
	TIMSK &= ~(_BV(OCIE2));
	_t2_func = f;
	OCR2 = ticks;
	//ASSR = 0;
	TCCR2 = _BV(WGM21) | prescaler;
	TCNT2 = 0;
	TIMSK |= _BV(OCIE2);
}

void timer2_stop() {
	TCCR2 = 0;
}

#ifdef ENABLE_TIMER2
ISR(TIMER2_COMPA_vect) {
	_t2_func();
}
#endif
