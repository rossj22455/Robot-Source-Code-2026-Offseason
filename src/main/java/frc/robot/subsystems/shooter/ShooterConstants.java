// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the shooter: drum flywheel (2x Falcon 500, master + inverted hardware follower,
 * onboard Phoenix 6 velocity control) and kicker (1x NEO, 3:1 gearbox). All CAN IDs, gains,
 * setpoints, and the distance-to-RPM map are PLACEHOLDERS to be populated during tuning.
 */
public class ShooterConstants {
  // CAN IDs — PLACEHOLDER assignments, but note the legal FRC CAN device ID range is 0-62;
  // out-of-range IDs are silently non-functional (drive uses 1-8, CANcoders 26-29, Pigeon 54)
  public static final int DRUM_MASTER_ID = 40;
  public static final int DRUM_FOLLOWER_ID = 41;
  public static final int KICKER_MOTOR_ID = 42;

  // Drum belt path: 12t pulley (Falcon) -> 105t pulley (intermediate) -> 15t pulley (drum).
  // With one pulley per shaft the intermediate 105t cancels out of the end-to-end ratio, so the
  // net reduction is 15/12 = 1.25 motor rotations per drum rotation (drum spins at 0.8x motor).
  // ASSUMPTION: if the 105t shaft actually carries a second (unstated) pulley, this ratio is
  // wrong — reverify against CAD before trusting shot speeds.
  public static final double DRUM_GEAR_RATIO = 15.0 / 12.0;

  // Drum physical model, from CAD mass properties (drum + wheels assembly: 3.6301 kg,
  // Lxx = 15.966 lb-in^2 about the spin axis). Radius is 2 in — the drum runs 4 in wheels.
  public static final double DRUM_MOI_KG_M2 = 0.00467;
  public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(2.0);

  // Drum onboard (Phoenix 6 device-level) velocity PID gains. kV is derived from the Falcon 500
  // free speed (6380 RPM = 106.3 rot/s at 12 V -> ~0.113 V per motor rot/s); kP is a gentle
  // starting value. PLACEHOLDER — retune on the real mechanism.
  public static final double DRUM_P = 0.1; // PLACEHOLDER starting gain
  public static final double DRUM_I = 0.0;
  public static final double DRUM_D = 0.0;
  public static final double DRUM_S = 0.0; // Static friction feedforward (volts)
  public static final double DRUM_V = 12.0 / 106.3; // Velocity FF (volts per motor rot/s)

  // Drum motor protection — PLACEHOLDER starting values
  public static final double DRUM_STATOR_CURRENT_LIMIT_AMPS = 60.0;
  public static final double DRUM_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

  // Vision distance (meters) -> drum RPM interpolation map. Starting values from the trajectory
  // calculator (64 degree hood, eta=0.9, exit height 17.973 in) — verify with real shot tuning.
  // NOTE: no ballistic solution exists below ~1.7 m at this hood angle; the interpolation map
  // clamps to the 1.7 m entry for closer shots. Rows are {distanceMeters, rpm}; distances MUST
  // be strictly increasing and distinct (validated at Shooter construction).
  public static final double[][] DISTANCE_TO_RPM_MAP = {
    {1.7, 1666.0},
    {2.2, 1714.0},
    {2.7, 1826.0},
    {3.2, 1947.0},
    {3.7, 2068.0},
    {4.3, 2211.0},
  };

  // isReadyToShoot(): drum velocity must stay within this tolerance of the vision-mapped target
  // for the debounce period — PLACEHOLDER (tighten during tuning; "razor-thin" per design)
  public static final double DRUM_READY_TOLERANCE_RPM = 50.0;
  public static final double DRUM_READY_DEBOUNCE_SECS = 0.1;

  // isReadyToShoot() also requires the robot heading to be within this tolerance of the hub
  // bearing (the hood is fixed, so aim IS heading). The hub radius (0.5969 m) subtends ~8 deg at
  // the longest table distance (4.3 m) — 5 deg leaves margin. PLACEHOLDER — tune on the field.
  public static final double AIM_TOLERANCE_DEG = 5.0;

  // Vision-loss fallback: vision cannot be relied on, so losing it must never disable the
  // shooter. With no recent vision correction the pose-derived distance is untrustworthy, so the
  // drum falls back to this fixed setpoint and the driver ranges by eye. PLACEHOLDER — pick the
  // RPM for the distance you most commonly shoot from.
  public static final double VISION_FALLBACK_RPM = 1900.0;

  // Neutral-zone funneling: fixed lob RPM toward the alliance corner (distance-to-corner varies
  // and precision doesn't matter — just get fuel back to friendly territory). PLACEHOLDER.
  public static final double FUNNEL_RPM = 2300.0;

  // Kicker (NEO through a 3:1 gearbox, basic voltage control) — PLACEHOLDER speeds
  public static final double KICKER_GEAR_RATIO = 3.0;
  public static final double KICKER_FEED_VOLTS = 6.0; // PLACEHOLDER — tune on real mechanism
  public static final double KICKER_REVERSE_VOLTS = -4.0; // PLACEHOLDER — tune on real mechanism
  public static final int KICKER_CURRENT_LIMIT_AMPS = 30;

  // Thermal monitoring — PLACEHOLDER warning threshold
  public static final double MOTOR_TEMP_WARNING_CELSIUS = 80.0;

  // --- Physical shot geometry (shared by the live trajectory prediction, which runs on the
  // real robot too, and by the simulation's launch model) ---
  // Fixed hood angle
  public static final double HOOD_ANGLE_DEG = 64.0;
  // Ball exit point from CAD: height above the floor, and XY offset from robot center
  public static final double BALL_EXIT_HEIGHT_METERS = Units.inchesToMeters(19.179766);
  public static final Translation2d BALL_EXIT_OFFSET =
      new Translation2d(Units.inchesToMeters(8.246225), 0.0);
  // Drum-surface-speed -> ball-exit-speed transfer ratio, back-solved from the RPM lookup table
  // so the table's RPMs physically drop into the hub (fit 0.63-0.67 across 1.7-4.3 m; vacuum
  // ballistics). NOT the trajectory calculator's eta.
  public static final double SURFACE_TO_BALL_SPEED_RATIO = 0.66;
}
