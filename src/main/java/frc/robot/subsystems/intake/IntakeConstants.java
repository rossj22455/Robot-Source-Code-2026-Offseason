// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

/**
 * Constants for the intake: linear rack articulation (1x NEO) and dual-roller bar (2x NEO, master +
 * inverted hardware follower). All tuning gains and physical setpoints are PLACEHOLDERS; protection
 * limits are conservative starting values to be validated on the real mechanism.
 */
public class IntakeConstants {
  // CAN IDs — PLACEHOLDER assignments, but note the legal FRC CAN device ID range is 0-62
  public static final int LINEAR_MOTOR_ID = 44;
  public static final int ROLLER_MASTER_ID = 45;
  public static final int ROLLER_FOLLOWER_ID = 46;

  // Linear mechanism gearing: 16t gear drives a 62t gear, which drives a 10 DP, 10t pinion on the
  // rack. Pinion pitch diameter = 10 teeth / 10 DP = 1.0 in, so the rack travels PI * 1.0 in per
  // pinion revolution. These values are derived from the mechanism, not tuned.
  public static final double LINEAR_GEAR_REDUCTION = 62.0 / 16.0; // Motor rotations per pinion rot
  public static final double LINEAR_PINION_PITCH_DIAMETER_METERS = Units.inchesToMeters(1.0);
  public static final double LINEAR_METERS_PER_MOTOR_ROTATION =
      (1.0 / LINEAR_GEAR_REDUCTION) * Math.PI * LINEAR_PINION_PITCH_DIAMETER_METERS;

  // Linear travel — the homing sequence defines 0.0 at the retracted hardstop. Full travel is
  // 11.498946 in hardstop-to-hardstop (from CAD). The working setpoints sit slightly inside the
  // hardstops so position control never slams the mechanical limits.
  public static final double LINEAR_MIN_POSITION_METERS = 0.0;
  public static final double LINEAR_MAX_POSITION_METERS = Units.inchesToMeters(11.498946);
  public static final double LINEAR_RETRACTED_POSITION_METERS = 0.003;
  public static final double LINEAR_EXTENDED_POSITION_METERS = LINEAR_MAX_POSITION_METERS - 0.005;

  // The rack extends out of the robot on a 23 degree incline from horizontal (from CAD). Used
  // for the 3D component visualization axis and the gravity component in simulation
  // (F = m * g * sin(angle) along the rack).
  public static final double RACK_ANGLE_RAD = Units.degreesToRadians(23.0);
  // From CAD mass properties: 10.863 lb carriage assembly (includes the NEO; excludes screws
  // and nuts, so the real value runs slightly heavier)
  public static final double CARRIAGE_MASS_KG = 4.93;

  // RIO-side profiled PID gains and constraints — PLACEHOLDER starting values (sim-workable;
  // retune on the real mechanism before first competition use)
  public static final double LINEAR_P = 40.0; // PLACEHOLDER
  public static final double LINEAR_I = 0.0;
  public static final double LINEAR_D = 0.0;
  public static final double LINEAR_MAX_VELOCITY_METERS_PER_SEC = 0.5; // PLACEHOLDER
  public static final double LINEAR_MAX_ACCELERATION_METERS_PER_SEC_SQ = 2.0; // PLACEHOLDER
  public static final double LINEAR_POSITION_TOLERANCE_METERS = 0.01; // PLACEHOLDER

  // NEO protection: REVLib smart current limits (amps). The holding limit is applied when
  // maintaining position against the retracted hardstop; the homing limit is applied while
  // intentionally stalling into the hardstop. PLACEHOLDER starting values.
  public static final int LINEAR_NORMAL_CURRENT_LIMIT_AMPS = 30;
  public static final int LINEAR_HOMING_CURRENT_LIMIT_AMPS = 10;
  public static final int LINEAR_HOLDING_CURRENT_LIMIT_AMPS = 5;

  // Boot-up homing: drive slowly into the retracted hardstop and detect the stall via current
  // rise + velocity stagnation. PLACEHOLDER values — verify direction (sign) and thresholds on
  // the real mechanism before first power-on.
  public static final double HOMING_VOLTS = -1.5;
  public static final double HOMING_STALL_CURRENT_AMPS = 8.0;
  public static final double HOMING_STALL_VELOCITY_METERS_PER_SEC = 0.005;
  public static final double HOMING_STALL_DEBOUNCE_SECS = 0.25;
  public static final double HOMING_TIMEOUT_SECS = 4.0;

  // Thermal monitoring — PLACEHOLDER warning threshold
  public static final double MOTOR_TEMP_WARNING_CELSIUS = 80.0;

  // Firing aggregation ("shutter"): while shooting, the intake sweeps fully in to herd balls,
  // then oscillates between the retracted position and this partially-out position until firing
  // stops. PLACEHOLDER stroke length — tune for effective ball agitation.
  public static final double SHUTTER_OUT_POSITION_METERS = 0.10;

  // Aggregation motion speed — deliberately slower than normal positioning so the intake herds
  // balls instead of batting them away. PLACEHOLDER — tune against real ball behavior.
  public static final double AGGREGATE_MAX_VELOCITY_METERS_PER_SEC = 0.15;
  public static final double AGGREGATE_MAX_ACCELERATION_METERS_PER_SEC_SQ = 0.8;

  // Rollers — PLACEHOLDER starting values: tune the intake/eject speeds on the real mechanism
  public static final double ROLLER_INTAKE_VOLTS = 8.0;
  public static final double ROLLER_EJECT_VOLTS = -6.0;
  public static final int ROLLER_CURRENT_LIMIT_AMPS = 40;
}
