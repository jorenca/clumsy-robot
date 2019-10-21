/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * stepper.cpp - A singleton object to execute motion plans using stepper motors
 * Marlin Firmware
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * Timer calculations informed by the 'RepRap cartesian firmware' by Zack Smith
 * and Philipp Tiefenbacher.
 */

/**
 *         __________________________
 *        /|                        |\     _________________         ^
 *       / |                        | \   /|               |\        |
 *      /  |                        |  \ / |               | \       s
 *     /   |                        |   |  |               |  \      p
 *    /    |                        |   |  |               |   \     e
 *   +-----+------------------------+---+--+---------------+----+    e
 *   |               BLOCK 1            |      BLOCK 2          |    d
 *
 *                           time ----->
 *
 *  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
 *  first block->accelerate_until step_events_completed, then keeps going at constant speed until
 *  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
 *  The slope of acceleration is calculated using v = u + at where t is the accumulated timer values of the steps so far.
 */

/**
 * Marlin uses the Bresenham algorithm. For a detailed explanation of theory and
 * method see https://www.cs.helsinki.fi/group/goa/mallinnus/lines/bresenh.html
 */

/**
 * Jerk controlled movements planner added Apr 2018 by Eduardo José Tagle.
 * Equations based on Synthethos TinyG2 sources, but the fixed-point
 * implementation is new, as we are running the ISR with a variable period.
 * Also implemented the Bézier velocity curve evaluation in ARM assembler,
 * to avoid impacting ISR speed.
 */

#include "Marlin.h"
#include "stepper.h"
#include "endstops.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"
#include "cardreader.h"
#include "speed_lookuptable.h"
#include "delay.h"


Stepper stepper; // Singleton

// public:


// private:

block_t* Stepper::current_block = NULL; // A pointer to the block currently being traced

uint8_t Stepper::last_direction_bits = 0,
        Stepper::axis_did_move;

bool Stepper::abort_current_block;

uint32_t Stepper::acceleration_time, Stepper::deceleration_time;
uint8_t Stepper::steps_per_isr;

  constexpr uint8_t Stepper::oversampling_factor;

int32_t Stepper::delta_error[NUM_AXIS] = { 0 };
uint32_t Stepper::advance_dividend[NUM_AXIS] = { 0 },
         Stepper::advance_divisor = 0,
         Stepper::step_events_completed = 0, // The number of step events executed in the current block
         Stepper::accelerate_until,          // The point from where we need to stop acceleration
         Stepper::decelerate_after,          // The point from where we need to start decelerating
         Stepper::step_event_count;          // The total event count for the current block


uint32_t Stepper::nextMainISR = 0;

  constexpr uint32_t LA_ADV_NEVER = 0xFFFFFFFF;
  uint32_t Stepper::nextAdvanceISR = LA_ADV_NEVER,
           Stepper::LA_isr_rate = LA_ADV_NEVER;
  uint16_t Stepper::LA_current_adv_steps = 0,
           Stepper::LA_final_adv_steps,
           Stepper::LA_max_adv_steps;

  int8_t   Stepper::LA_steps = 0;

  bool Stepper::LA_use_advance_lead;


int32_t Stepper::ticks_nominal = -1;

  uint32_t Stepper::acc_step_rate; // needed for deceleration start point

volatile int32_t Stepper::endstops_trigsteps[XYZ],
                 Stepper::count_position[NUM_AXIS] = { 0 };
int8_t Stepper::count_direction[NUM_AXIS] = {
  1, 1, 1, 1
};


  #define X_APPLY_DIR(v,Q) X_DIR_WRITE(v)
  #define X_APPLY_STEP(v,Q) X_STEP_WRITE(v)

  #define Y_APPLY_DIR(v,Q) Y_DIR_WRITE(v)
  #define Y_APPLY_STEP(v,Q) Y_STEP_WRITE(v)



// intRes = longIn1 * longIn2 >> 24
// uses:
// A[tmp] to store 0
// B[tmp] to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
// B A are bits 24-39 and are the returned value
// C B A is longIn1
// D C B A is longIn2
//
static FORCE_INLINE uint16_t MultiU24X32toH16(uint32_t longIn1, uint32_t longIn2) {
  register uint8_t tmp1;
  register uint8_t tmp2;
  register uint16_t intRes;
  __asm__ __volatile__(
    A("clr %[tmp1]")
    A("mul %A[longIn1], %B[longIn2]")
    A("mov %[tmp2], r1")
    A("mul %B[longIn1], %C[longIn2]")
    A("movw %A[intRes], r0")
    A("mul %C[longIn1], %C[longIn2]")
    A("add %B[intRes], r0")
    A("mul %C[longIn1], %B[longIn2]")
    A("add %A[intRes], r0")
    A("adc %B[intRes], r1")
    A("mul %A[longIn1], %C[longIn2]")
    A("add %[tmp2], r0")
    A("adc %A[intRes], r1")
    A("adc %B[intRes], %[tmp1]")
    A("mul %B[longIn1], %B[longIn2]")
    A("add %[tmp2], r0")
    A("adc %A[intRes], r1")
    A("adc %B[intRes], %[tmp1]")
    A("mul %C[longIn1], %A[longIn2]")
    A("add %[tmp2], r0")
    A("adc %A[intRes], r1")
    A("adc %B[intRes], %[tmp1]")
    A("mul %B[longIn1], %A[longIn2]")
    A("add %[tmp2], r1")
    A("adc %A[intRes], %[tmp1]")
    A("adc %B[intRes], %[tmp1]")
    A("lsr %[tmp2]")
    A("adc %A[intRes], %[tmp1]")
    A("adc %B[intRes], %[tmp1]")
    A("mul %D[longIn2], %A[longIn1]")
    A("add %A[intRes], r0")
    A("adc %B[intRes], r1")
    A("mul %D[longIn2], %B[longIn1]")
    A("add %B[intRes], r0")
    A("clr r1")
      : [intRes] "=&r" (intRes),
        [tmp1] "=&r" (tmp1),
        [tmp2] "=&r" (tmp2)
      : [longIn1] "d" (longIn1),
        [longIn2] "d" (longIn2)
      : "cc"
  );
  return intRes;
}

void Stepper::wake_up() {
  // TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

/**
 * Set the stepper direction of each axis
 *
 *   COREXY: X_AXIS=A_AXIS and Y_AXIS=B_AXIS
 *   COREXZ: X_AXIS=A_AXIS and Z_AXIS=C_AXIS
 *   COREYZ: Y_AXIS=B_AXIS and Z_AXIS=C_AXIS
 */
void Stepper::set_directions() {

  #define SET_STEP_DIR(A) \
    if (motor_direction(_AXIS(A))) { \
      A##_APPLY_DIR(INVERT_## A##_DIR, false); \
      count_direction[_AXIS(A)] = -1; \
    } \
    else { \
      A##_APPLY_DIR(!INVERT_## A##_DIR, false); \
      count_direction[_AXIS(A)] = 1; \
    }

  #if HAS_X_DIR
    SET_STEP_DIR(X); // A
  #endif
  #if HAS_Y_DIR
    SET_STEP_DIR(Y); // B
  #endif

  // A small delay may be needed after changing direction
  #if MINIMUM_STEPPER_DIR_DELAY > 0
    DELAY_NS(MINIMUM_STEPPER_DIR_DELAY);
  #endif
}

/**
 * Stepper Driver Interrupt
 *
 * Directly pulses the stepper motors at high frequency.
 */

HAL_STEP_TIMER_ISR {
  HAL_timer_isr_prologue(STEP_TIMER_NUM);

  Stepper::isr();

  HAL_timer_isr_epilogue(STEP_TIMER_NUM);
}

#define STEP_MULTIPLY(A,B) MultiU24X32toH16(A, B)

void Stepper::isr() {
  DISABLE_ISRS();

  // Program timer compare for the maximum period, so it does NOT
  // flag an interrupt while this ISR is running - So changes from small
  // periods to big periods are respected and the timer does not reset to 0
  HAL_timer_set_compare(STEP_TIMER_NUM, HAL_TIMER_TYPE_MAX);

  // Count of ticks for the next ISR
  hal_timer_t next_isr_ticks = 0;

  // Limit the amount of iterations
  uint8_t max_loops = 10;

  // We need this variable here to be able to use it in the following loop
  hal_timer_t min_ticks;
  do {
    // Enable ISRs to reduce USART processing latency
    ENABLE_ISRS();

    // Run main stepping pulse phase ISR if we have to
    if (!nextMainISR) Stepper::stepper_pulse_phase_isr();

      // Run linear advance stepper ISR if we have to
      if (!nextAdvanceISR) nextAdvanceISR = Stepper::advance_isr();

    // ^== Time critical. NOTHING besides pulse generation should be above here!!!

    // Run main stepping block processing ISR if we have to
    if (!nextMainISR) nextMainISR = Stepper::stepper_block_phase_isr();

    uint32_t interval = MIN(nextAdvanceISR, nextMainISR);  // Nearest time interval

    // Limit the value to the maximum possible value of the timer
    NOMORE(interval, HAL_TIMER_TYPE_MAX);

    // Compute the time remaining for the main isr
    nextMainISR -= interval;

      // Compute the time remaining for the advance isr
      if (nextAdvanceISR != LA_ADV_NEVER) nextAdvanceISR -= interval;

    /**
     * This needs to avoid a race-condition caused by interleaving
     * of interrupts required by both the LA and Stepper algorithms.
     *
     * Assume the following tick times for stepper pulses:
     *   Stepper ISR (S):  1 1000 2000 3000 4000
     *   Linear Adv. (E): 10 1010 2010 3010 4010
     *
     * The current algorithm tries to interleave them, giving:
     *  1:S 10:E 1000:S 1010:E 2000:S 2010:E 3000:S 3010:E 4000:S 4010:E
     *
     * Ideal timing would yield these delta periods:
     *  1:S  9:E  990:S   10:E  990:S   10:E  990:S   10:E  990:S   10:E
     *
     * But, since each event must fire an ISR with a minimum duration, the
     * minimum delta might be 900, so deltas under 900 get rounded up:
     *  900:S d900:E d990:S d900:E d990:S d900:E d990:S d900:E d990:S d900:E
     *
     * It works, but divides the speed of all motors by half, leading to a sudden
     * reduction to 1/2 speed! Such jumps in speed lead to lost steps (not even
     * accounting for double/quad stepping, which makes it even worse).
     */

    // Compute the tick count for the next ISR
    next_isr_ticks += interval;

    /**
     * The following section must be done with global interrupts disabled.
     * We want nothing to interrupt it, as that could mess the calculations
     * we do for the next value to program in the period register of the
     * stepper timer and lead to skipped ISRs (if the value we happen to program
     * is less than the current count due to something preempting between the
     * read and the write of the new period value).
     */
    DISABLE_ISRS();

    /**
     * Get the current tick value + margin
     * Assuming at least 6µs between calls to this ISR...
     * On AVR the ISR epilogue+prologue is estimated at 100 instructions - Give 8µs as margin
     * On ARM the ISR epilogue+prologue is estimated at 20 instructions - Give 1µs as margin
     */
    min_ticks = HAL_timer_get_count(STEP_TIMER_NUM) + hal_timer_t((STEPPER_TIMER_TICKS_PER_US) * 8);

    /**
     * NB: If for some reason the stepper monopolizes the MPU, eventually the
     * timer will wrap around (and so will 'next_isr_ticks'). So, limit the
     * loop to 10 iterations. Beyond that, there's no way to ensure correct pulse
     * timing, since the MCU isn't fast enough.
     */
    if (!--max_loops) next_isr_ticks = min_ticks;

    // Advance pulses if not enough time to wait for the next ISR
  } while (next_isr_ticks < min_ticks);

  // Now 'next_isr_ticks' contains the period to the next Stepper ISR - And we are
  // sure that the time has not arrived yet - Warrantied by the scheduler

  // Set the next ISR to fire at the proper time
  HAL_timer_set_compare(STEP_TIMER_NUM, hal_timer_t(next_isr_ticks));

  // Don't forget to finally reenable interrupts
  ENABLE_ISRS();
}

/**
 * This phase of the ISR should ONLY create the pulses for the steppers.
 * This prevents jitter caused by the interval between the start of the
 * interrupt and the start of the pulses. DON'T add any logic ahead of the
 * call to this method that might cause variation in the timing. The aim
 * is to keep pulse timing as regular as possible.
 */
//#if ENABLED(UNREGISTERED_MOVE_SUPPORT)
  #define COUNT_IT current_block->count_it
//#else
  //#define COUNT_IT true
//#endif

void Stepper::stepper_pulse_phase_isr() {

  // If we must abort the current block, do so!
  if (abort_current_block) {
    abort_current_block = false;
    if (current_block) {
      axis_did_move = 0;
      current_block = NULL;
      planner.discard_current_block();
    }
  }

  // If there is no current block, do nothing
  if (!current_block) return;

  // Count of pending loops and events for this iteration
  const uint32_t pending_events = step_event_count - step_events_completed;
  uint8_t events_to_do = MIN(pending_events, steps_per_isr);

  // Just update the value we will get at the end of the loop
  step_events_completed += events_to_do;

  // Get the timer count and estimate the end of the pulse
  hal_timer_t pulse_end = HAL_timer_get_count(PULSE_TIMER_NUM) + hal_timer_t(MIN_PULSE_TICKS);

  const hal_timer_t added_step_ticks = hal_timer_t(ADDED_STEP_TICKS);

  // Take multiple steps per interrupt (For high speed moves)
  do {

    #define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
    #define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN

    // Start an active pulse, if Bresenham says so, and update position
    #define PULSE_START(AXIS) do{ \
      delta_error[_AXIS(AXIS)] += advance_dividend[_AXIS(AXIS)]; \
      if (delta_error[_AXIS(AXIS)] >= 0) { \
        _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS), 0); \
        if (COUNT_IT) count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
      } \
    }while(0)

    // Stop an active pulse, if any, and adjust error term
    #define PULSE_STOP(AXIS) do { \
      if (delta_error[_AXIS(AXIS)] >= 0) { \
        delta_error[_AXIS(AXIS)] -= advance_divisor; \
        _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS), 0); \
      } \
    }while(0)

    // Pulse start

    #if MINIMUM_STEPPER_PULSE
      // Just wait for the requested pulse duration
      while (HAL_timer_get_count(PULSE_TIMER_NUM) < pulse_end) { /* nada */ }
    #endif

    // Add the delay needed to ensure the maximum driver rate is enforced
    if (signed(added_step_ticks) > 0) pulse_end += hal_timer_t(added_step_ticks);

      #if HAS_X_STEP
        PULSE_STOP(X);
      #endif
      #if HAS_Y_STEP
        PULSE_STOP(Y);
      #endif

    // Decrement the count of pending pulses to do
    --events_to_do;

    // For minimum pulse time wait after stopping pulses also
    if (events_to_do) {
      // Just wait for the requested pulse duration
      while (HAL_timer_get_count(PULSE_TIMER_NUM) < pulse_end) { /* nada */ }
      #if MINIMUM_STEPPER_PULSE
        // Add to the value, the time that the pulse must be active (to be used on the next loop)
        pulse_end += hal_timer_t(MIN_PULSE_TICKS);
      #endif
    }

  } while (events_to_do);
}

// This is the last half of the stepper interrupt: This one processes and
// properly schedules blocks from the planner. This is executed after creating
// the step pulses, so it is not time critical, as pulses are already done.

uint32_t Stepper::stepper_block_phase_isr() {

  // If no queued movements, just wait 1ms for the next move
  uint32_t interval = (STEPPER_TIMER_RATE / 1000);

  // If there is a current block
  if (current_block) {

    // If current block is finished, reset pointer
    if (step_events_completed >= step_event_count) {
      axis_did_move = 0;
      current_block = NULL;
      planner.discard_current_block();
    }
    else {
      // Step events not completed yet...

      // Are we in acceleration phase ?
      if (step_events_completed <= accelerate_until) { // Calculate new timer value

          acc_step_rate = STEP_MULTIPLY(acceleration_time, current_block->acceleration_rate) + current_block->initial_rate;
          NOMORE(acc_step_rate, current_block->nominal_rate);
        // acc_step_rate is in steps/second

        // step_rate to timer interval and steps per stepper isr
        interval = calc_timer_interval(acc_step_rate, oversampling_factor, &steps_per_isr);
        acceleration_time += interval;

          if (LA_use_advance_lead) {
            // Fire ISR if final adv_rate is reached
            if (LA_steps && LA_isr_rate != current_block->advance_speed) nextAdvanceISR = 0;
          }
          else if (LA_steps) nextAdvanceISR = 0;
      }
      // Are we in Deceleration phase ?
      else if (step_events_completed > decelerate_after) {
        uint32_t step_rate;

          // Using the old trapezoidal control
          step_rate = STEP_MULTIPLY(deceleration_time, current_block->acceleration_rate);
          if (step_rate < acc_step_rate) { // Still decelerating?
            step_rate = acc_step_rate - step_rate;
            NOLESS(step_rate, current_block->final_rate);
          }
          else
            step_rate = current_block->final_rate;

        // step_rate is in steps/second

        // step_rate to timer interval and steps per stepper isr
        interval = calc_timer_interval(step_rate, oversampling_factor, &steps_per_isr);
        deceleration_time += interval;

          if (LA_use_advance_lead) {
            // Wake up eISR on first deceleration loop and fire ISR if final adv_rate is reached
            if (step_events_completed <= decelerate_after + steps_per_isr || (LA_steps && LA_isr_rate != current_block->advance_speed)) {
              nextAdvanceISR = 0;
              LA_isr_rate = current_block->advance_speed;
            }
          }
          else if (LA_steps) nextAdvanceISR = 0;
      }
      // We must be in cruise phase otherwise
      else {

          // If there are any esteps, fire the next advance_isr "now"
          if (LA_steps && LA_isr_rate != current_block->advance_speed) nextAdvanceISR = 0;

        // Calculate the ticks_nominal for this nominal speed, if not done yet
        if (ticks_nominal < 0) {
          // step_rate to timer interval and loops for the nominal speed
          ticks_nominal = calc_timer_interval(current_block->nominal_rate, oversampling_factor, &steps_per_isr);
        }

        // The timer interval is just the nominal value for the nominal speed
        interval = ticks_nominal;
      }
    }
  }

  // If there is no current block at this point, attempt to pop one from the buffer
  // and prepare its movement
  if (!current_block) {

    // Anything in the buffer?
    if ((current_block = planner.get_current_block())) {

      // Sync block? Sync the stepper counts and return
      while (TEST(current_block->flag, BLOCK_BIT_SYNC_POSITION)) {
        _set_position(
          current_block->position[A_AXIS], current_block->position[B_AXIS], current_block->position[C_AXIS],

          current_block->position[E_AXIS]
        );
        planner.discard_current_block();

        // Try to get a new block
        if (!(current_block = planner.get_current_block()))
          return interval; // No more queued movements!
      }

      // Flag all moving axes for proper endstop handling

      #if IS_CORE
        // Define conditions for checking endstops
        #define S_(N) current_block->steps[CORE_AXIS_##N]
        #define D_(N) TEST(current_block->direction_bits, CORE_AXIS_##N)
      #endif


      #define X_MOVE_TEST !!current_block->steps[A_AXIS]
      #define Y_MOVE_TEST !!current_block->steps[B_AXIS]

      uint8_t axis_bits = 0;
      if (X_MOVE_TEST) SBI(axis_bits, A_AXIS);
      if (Y_MOVE_TEST) SBI(axis_bits, B_AXIS);
      //if (!!current_block->steps[A_AXIS]) SBI(axis_bits, X_HEAD);
      //if (!!current_block->steps[B_AXIS]) SBI(axis_bits, Y_HEAD);
      //if (!!current_block->steps[C_AXIS]) SBI(axis_bits, Z_HEAD);
      axis_did_move = axis_bits;

      // No acceleration / deceleration time elapsed so far
      acceleration_time = deceleration_time = 0;

      uint8_t oversampling = 0;                         // Assume we won't use it

      // Based on the oversampling factor, do the calculations
      step_event_count = current_block->step_event_count << oversampling;

      // Initialize Bresenham delta errors to 1/2
        delta_error[X_AXIS] = delta_error[Y_AXIS] = -int32_t(step_event_count);

      // Calculate Bresenham dividends
      advance_dividend[X_AXIS] = current_block->steps[X_AXIS] << 1;
      advance_dividend[Y_AXIS] = current_block->steps[Y_AXIS] << 1;

      // Calculate Bresenham divisor
      advance_divisor = step_event_count << 1;

      // No step events completed so far
      step_events_completed = 0;

      // Compute the acceleration and deceleration points
      accelerate_until = current_block->accelerate_until << oversampling;
      decelerate_after = current_block->decelerate_after << oversampling;

      // Initialize the trapezoid generator from the current block.

        if ((LA_use_advance_lead = current_block->use_advance_lead)) {
          LA_final_adv_steps = current_block->final_adv_steps;
          LA_max_adv_steps = current_block->max_adv_steps;
          //Start the ISR
          nextAdvanceISR = 0;
          LA_isr_rate = current_block->advance_speed;
        }
        else LA_isr_rate = LA_ADV_NEVER;

      if (current_block->direction_bits != last_direction_bits) {
        last_direction_bits = current_block->direction_bits;
        set_directions();
      }

      // Mark the time_nominal as not calculated yet
      ticks_nominal = -1;

        // Set as deceleration point the initial rate of the block
        acc_step_rate = current_block->initial_rate;

      // Calculate the initial timer interval
      interval = calc_timer_interval(current_block->initial_rate, oversampling_factor, &steps_per_isr);
    }
  }

  // Return the interval to wait
  return interval;
}

// Timer interrupt for E. LA_steps is set in the main routine
uint32_t Stepper::advance_isr() {
    uint32_t interval;

    if (LA_use_advance_lead) {
      if (step_events_completed > decelerate_after && LA_current_adv_steps > LA_final_adv_steps) {
        LA_steps--;
        LA_current_adv_steps--;
        interval = LA_isr_rate;
      }
      else if (step_events_completed < decelerate_after && LA_current_adv_steps < LA_max_adv_steps) {
             //step_events_completed <= (uint32_t)accelerate_until) {
        LA_steps++;
        LA_current_adv_steps++;
        interval = LA_isr_rate;
      }
      else
        interval = LA_isr_rate = LA_ADV_NEVER;
    }
    else
      interval = LA_ADV_NEVER;

    // Get the timer count and estimate the end of the pulse
    hal_timer_t pulse_end = HAL_timer_get_count(PULSE_TIMER_NUM) + hal_timer_t(MIN_PULSE_TICKS);

    const hal_timer_t added_step_ticks = hal_timer_t(ADDED_STEP_TICKS);

    // Step E stepper if we have steps
    while (LA_steps) {

      // Set the STEP pulse ON


      // Enforce a minimum duration for STEP pulse ON
      #if MINIMUM_STEPPER_PULSE
        // Just wait for the requested pulse duration
        while (HAL_timer_get_count(PULSE_TIMER_NUM) < pulse_end) { /* nada */ }
      #endif

      // Add the delay needed to ensure the maximum driver rate is enforced
      if (signed(added_step_ticks) > 0) pulse_end += hal_timer_t(added_step_ticks);

      LA_steps < 0 ? ++LA_steps : --LA_steps;

      // Set the STEP pulse OFF

      // For minimum pulse time wait before looping
      // Just wait for the requested pulse duration
      if (LA_steps) {
        while (HAL_timer_get_count(PULSE_TIMER_NUM) < pulse_end) { /* nada */ }
        #if MINIMUM_STEPPER_PULSE
          // Add to the value, the time that the pulse must be active (to be used on the next loop)
          pulse_end += hal_timer_t(MIN_PULSE_TICKS);
        #endif
      }
    } // LA_steps

    return interval;
  }
#endif // LIN_ADVANCE

// Check if the given block is busy or not - Must not be called from ISR contexts
// The current_block could change in the middle of the read by an Stepper ISR, so
// we must explicitly prevent that!
bool Stepper::is_block_busy(const block_t* const block) {
  #define sw_barrier() asm volatile("": : :"memory");

  // Keep reading until 2 consecutive reads return the same value,
  // meaning there was no update in-between caused by an interrupt.
  // This works because stepper ISRs happen at a slower rate than
  // successive reads of a variable, so 2 consecutive reads with
  // the same value means no interrupt updated it.
  block_t* vold, *vnew = current_block;
  sw_barrier();
  do {
    vold = vnew;
    vnew = current_block;
    sw_barrier();
  } while (vold != vnew);

  // Return if the block is busy or not
  return block == vnew;
}

void Stepper::init() {
  // Init Dir Pins
  X_DIR_INIT;
  Y_DIR_INIT;

  // Init Enable Pins - steppers default to disabled.
  X_ENABLE_INIT;
  if (!X_ENABLE_ON) X_ENABLE_WRITE(HIGH);
  Y_ENABLE_INIT;
  if (!Y_ENABLE_ON) Y_ENABLE_WRITE(HIGH);

  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE(AXIS) disable_## AXIS()

  #define AXIS_INIT(AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, _INVERT_STEP_PIN(PIN)); \
    _DISABLE(AXIS)


  // Init Step Pins
  AXIS_INIT(X, X);
  AXIS_INIT(Y, Y);

  // Init Stepper ISR to 122 Hz for quick starting
  HAL_timer_start(STEP_TIMER_NUM, 122); // OCR1A = 0x4000

  ENABLE_STEPPER_DRIVER_INTERRUPT();

  sei();

  set_directions(); // Init directions to last_direction_bits = 0
}

/**
 * Set the stepper positions directly in steps
 *
 * The input is based on the typical per-axis XYZ steps.
 * For CORE machines XYZ needs to be translated to ABC.
 *
 * This allows get_axis_position_mm to correctly
 * derive the current XYZ position later on.
 */
void Stepper::_set_position(const int32_t &x, const int32_t &y) {
    count_position[X_AXIS] = x;
    count_position[Y_AXIS] = y;
}

/**
 * Get a stepper's position in steps.
 */
int32_t Stepper::position(const AxisEnum axis) {
  const bool was_enabled = STEPPER_ISR_ENABLED();
  if (was_enabled) DISABLE_STEPPER_DRIVER_INTERRUPT();

  const int32_t v = count_position[axis];

  if (was_enabled) ENABLE_STEPPER_DRIVER_INTERRUPT();
  return v;
}

// Signal endstops were triggered - This function can be called from
// an ISR context  (Temperature, Stepper or limits ISR), so we must
// be very careful here. If the interrupt being preempted was the
// Stepper ISR (this CAN happen with the endstop limits ISR) then
// when the stepper ISR resumes, we must be very sure that the movement
// is properly cancelled
void Stepper::endstop_triggered(const AxisEnum axis) {

  const bool was_enabled = STEPPER_ISR_ENABLED();
  if (was_enabled) DISABLE_STEPPER_DRIVER_INTERRUPT();

  // Discard the rest of the move if there is a current block
  quick_stop();

  if (was_enabled) ENABLE_STEPPER_DRIVER_INTERRUPT();
}

int32_t Stepper::triggered_position(const AxisEnum axis) {
  const bool was_enabled = STEPPER_ISR_ENABLED();
  if (was_enabled) DISABLE_STEPPER_DRIVER_INTERRUPT();

  const int32_t v = endstops_trigsteps[axis];

  if (was_enabled) ENABLE_STEPPER_DRIVER_INTERRUPT();

  return v;
}

void Stepper::report_positions() {
  // Protect the access to the position.
  const bool was_enabled = STEPPER_ISR_ENABLED();
  if (was_enabled) DISABLE_STEPPER_DRIVER_INTERRUPT();

  const int32_t xpos = count_position[X_AXIS],
                ypos = count_position[Y_AXIS];

  if (was_enabled) ENABLE_STEPPER_DRIVER_INTERRUPT();

  SERIAL_PROTOCOLPGM(MSG_COUNT_X);
  SERIAL_PROTOCOL(xpos);

  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(ypos);

  SERIAL_EOL();
}
