// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.Region2d;
import java.util.Optional;

/**
 * Field zones and targeting points for the 2026 REBUILT game. Shooting at the hub from inside the
 * neutral zone is illegal; there, fuel is funneled toward the robot's own alliance corner instead
 * (never across toward the opposing alliance).
 */
public class FieldConstants {
  // Neutral zone halves (field-absolute coordinates; "RIGHT" is the low-Y half as seen from the
  // blue driver station). Bounds intentionally overshoot the field edges slightly for robustness.
  public static final Region2d NEUTRAL_ZONE_RIGHT =
      new Region2d(
          new Translation2d[] {
            new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(-10)), // bottom-left
            new Translation2d(Units.inchesToMeters(445), Units.inchesToMeters(-10)), // bottom-right
            new Translation2d(Units.inchesToMeters(445), Units.inchesToMeters(159)), // top-right
            new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(159)) // top-left
          });

  public static final Region2d NEUTRAL_ZONE_LEFT =
      new Region2d(
          new Translation2d[] {
            new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(159)),
            new Translation2d(Units.inchesToMeters(445), Units.inchesToMeters(159)),
            new Translation2d(Units.inchesToMeters(445), Units.inchesToMeters(330)),
            new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(330))
          });

  // Funnel aim points: just inside each corner of the robot's OWN alliance wall, so neutral-zone
  // fuel always travels back toward friendly territory. PLACEHOLDER positions (~1 m inside the
  // corners of the 651 x 318 in field) — tune against the real field.
  public static final Translation2d BLUE_FUNNEL_TARGET_LOW_Y = new Translation2d(1.0, 1.0);
  public static final Translation2d BLUE_FUNNEL_TARGET_HIGH_Y = new Translation2d(1.0, 7.07);
  public static final Translation2d RED_FUNNEL_TARGET_LOW_Y = new Translation2d(15.54, 1.0);
  public static final Translation2d RED_FUNNEL_TARGET_HIGH_Y = new Translation2d(15.54, 7.07);

  /**
   * Returns the corner point to funnel fuel toward when the robot is inside the neutral zone, or
   * empty when hub shooting is legal. The corner is always on the robot's own alliance wall, on the
   * same side of the field as the robot (shortest legal path home).
   */
  public static Optional<Translation2d> getFunnelTarget(Translation2d robotPosition) {
    boolean lowSide;
    if (NEUTRAL_ZONE_RIGHT.contains(robotPosition)) {
      lowSide = true;
    } else if (NEUTRAL_ZONE_LEFT.contains(robotPosition)) {
      lowSide = false;
    } else {
      return Optional.empty();
    }

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    if (isRed) {
      return Optional.of(lowSide ? RED_FUNNEL_TARGET_LOW_Y : RED_FUNNEL_TARGET_HIGH_Y);
    } else {
      return Optional.of(lowSide ? BLUE_FUNNEL_TARGET_LOW_Y : BLUE_FUNNEL_TARGET_HIGH_Y);
    }
  }
}
