/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * motion.cpp
 */

#include "motion.h"
#include "stepper.h"
#include "planner.h"
#include "../inc/MarlinConfig.h"

#define XYZ_CONSTS(type, array, CONFIG) const PROGMEM type array##_P[XYZ] = { X_##CONFIG, Y_##CONFIG, Z_##CONFIG }

XYZ_CONSTS(float, base_min_pos,   MIN_POS);
XYZ_CONSTS(float, base_max_pos,   MAX_POS);
XYZ_CONSTS(float, base_home_pos,  HOME_POS);
XYZ_CONSTS(float, max_length,     MAX_LENGTH);
XYZ_CONSTS(float, home_bump_mm,   HOME_BUMP_MM);
XYZ_CONSTS(signed char, home_dir, HOME_DIR);

/**
 * axis_homed
 *   Flags that each linear axis was homed.
 *   XYZ on cartesian, ABC on delta, ABZ on SCARA.
 *
 * axis_known_position
 *   Flags that the position is known in each linear axis. Set when homed.
 *   Cleared whenever a stepper powers off, potentially losing its position.
 */
uint8_t axis_homed, axis_known_position; // = 0

// Relative Mode. Enable with G91, disable with G90.
bool relative_mode; // = false;

/**
 * Cartesian Current Position
 *   Used to track the native machine position as moves are queued.
 *   Used by 'line_to_current_position' to do a move after changing it.
 *   Used by 'sync_plan_position' to update 'planner.position'.
 */
#define XY 2
#define X_AXIS 0
#define Y_AXIS 0
float current_position[XY] = { X_HOME_POS, Y_HOME_POS };

/**
 * Cartesian Destination
 *   The destination for a move, filled in by G-code movement commands,
 *   and expected by functions like 'prepare_move_to_destination'.
 *   Set with 'get_destination_from_command' or 'set_destination_from_current'.
 */
float destination[XY]; // = { 0 }



// The feedrate for the current move, often used as the default if
// no other feedrate is specified. Overridden for special moves.
// Set by the last G0 through G5 command's "F" parameter.
// Functions that override this for custom moves *must always* restore it!
float feedrate_mm_s = MMM_TO_MMS(1500.0f);

int16_t feedrate_percentage = 100;

// Homing feedrate is const progmem - compare to constexpr in the header
const float homing_feedrate_mm_s[XY] PROGMEM = {
  MMM_TO_MMS(HOMING_FEEDRATE_XY),
  MMM_TO_MMS(HOMING_FEEDRATE_XY)
};

// Cartesian conversion result goes here:
float cartes[XY];

/**
 * Output the current position to serial
 */
void report_current_position() {
  SERIAL_ECHOPAIR("X:", LOGICAL_X_POSITION(current_position[X_AXIS]));
  SERIAL_ECHOPAIR(" Y:", LOGICAL_Y_POSITION(current_position[Y_AXIS]));

  stepper.report_positions();
}

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void sync_plan_position() {
  planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

/**
 * Get the stepper positions in the cartes[] array.
 * Forward kinematics are applied for DELTA and SCARA.
 *
 * The result is in the current coordinate space with
 * leveling applied. The coordinates need to be run through
 * unapply_leveling to obtain the "ideal" coordinates
 * suitable for current_position, etc.
 */
void get_cartesian_from_steppers() {
    cartes[X_AXIS] = planner.get_axis_position_mm(X_AXIS);
    cartes[Y_AXIS] = planner.get_axis_position_mm(Y_AXIS);
  #endif
}

/**
 * Set the current_position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 *
 * To prevent small shifts in axis position always call
 * sync_plan_position after updating axes with this.
 *
 * To keep hosts in sync, always call report_current_position
 * after updating the current_position.
 */
void set_current_from_steppers_for_axis(const AxisEnum axis) {
  get_cartesian_from_steppers();
  current_position[axis] = cartes[axis];
}

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
void line_to_current_position(const float &fr_mm_s/*=feedrate_mm_s*/) {
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], fr_mm_s);
}

/**
 * Move the planner to the position stored in the destination array, which is
 * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
 */
void buffer_line_to_destination(const float fr_mm_s) {
  planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], fr_mm_s);
}

/**
 * Plan a move to (X, Y, Z) and set the current_position
 */
void do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s/*=0.0*/) {
  const float xy_feedrate = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;

    current_position[X_AXIS] = rx;
    current_position[Y_AXIS] = ry;
    line_to_current_position(xy_feedrate);

  planner.synchronize();
}
void do_blocking_move_to_x(const float &rx, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(rx, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
}
void do_blocking_move_to_xy(const float &rx, const float &ry, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(rx, ry, current_position[Z_AXIS], fr_mm_s);
}

//
// Prepare to do endstop or probe moves with custom feedrates.
//  - Save / restore current feedrate and multiplier
//
static float saved_feedrate_mm_s;
static int16_t saved_feedrate_percentage;
void setup_for_endstop_or_probe_move() {
  saved_feedrate_mm_s = feedrate_mm_s;
  saved_feedrate_percentage = feedrate_percentage;
  feedrate_percentage = 100;
}
void clean_up_after_endstop_or_probe_move() {
  feedrate_mm_s = saved_feedrate_mm_s;
  feedrate_percentage = saved_feedrate_percentage;
}


/**
 * Prepare a linear move in a Cartesian setup.
 *
 * When a mesh-based leveling system is active, moves are segmented
 * according to the configuration of the leveling system.
 *
 * Returns true if current_position[] was set to destination[]
 */
inline void prepare_move_to_destination_cartesian() {
  buffer_line_to_destination(MMS_SCALED(feedrate_mm_s));
  // caller will update current_position
}

/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 *
 * Make sure current_position[E] and destination[E] are good
 * before calling or cold/lengthy extrusion may get missed.
 */
void prepare_move_to_destination() {
  apply_motion_limits(destination);

  prepare_move_to_destination_cartesian();

  set_current_from_destination();
}

/**
 * Set an axis' current position to its home position (after homing).
 *
 * For Core and Cartesian robots this applies one-to-one when an
 * individual axis has been homed.
 *
 * DELTA should wait until all homing is done before setting the XYZ
 * current_position to home, because homing is a single operation.
 * In the case where the axis positions are already known and previously
 * homed, DELTA could home to X or Y individually by moving either one
 * to the center. However, homing Z always homes XY and Z.
 *
 * SCARA should wait until all XY homing is done before setting the XY
 * current_position to home, because neither X nor Y is at home until
 * both are at home. Z can however be homed individually.
 *
 * Callers must sync the planner position after calling this!
 */
void set_axis_is_at_home(const AxisEnum axis) {
  SBI(axis_known_position, axis);
  SBI(axis_homed, axis);

  current_position[axis] = base_home_pos(axis);
}

/**
 * Set an axis' to be unhomed.
 */
void set_axis_is_not_at_home(const AxisEnum axis) {
  CBI(axis_known_position, axis);
  CBI(axis_homed, axis);
}
