// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

/**
 * Constants for autonomous tilt-anomaly detection (Pigeon 2 pitch/roll monitoring) and the dynamic
 * recovery maneuver. All values are PLACEHOLDERS to be tuned against the real robot's behavior on
 * field elements.
 */
public class TiltConstants {
  // Detection: |pitch| or |roll| beyond this (sustained for the debounce) is a tilt anomaly
  public static final double TILT_THRESHOLD_DEGREES = 10.0; // PLACEHOLDER
  public static final double TILT_DEBOUNCE_SECS = 0.1; // PLACEHOLDER

  // Recovery: brief robot-relative reverse maneuver to dislodge
  public static final double RECOVERY_REVERSE_SPEED_METERS_PER_SEC = 1.0; // PLACEHOLDER
  public static final double RECOVERY_REVERSE_DURATION_SECS = 0.5; // PLACEHOLDER

  // On-the-fly pathfinding constraints for routing back to the next waypoint — PLACEHOLDER
  // (intentionally slower than normal auto speeds for a safe recovery)
  public static final double RECOVERY_MAX_VELOCITY_METERS_PER_SEC = 2.0;
  public static final double RECOVERY_MAX_ACCELERATION_METERS_PER_SEC_SQ = 2.0;
  public static final double RECOVERY_MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 360.0;
  public static final double RECOVERY_MAX_ANGULAR_ACCELERATION_DEG_PER_SEC_SQ = 540.0;
}
