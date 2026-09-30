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
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.Region2d;
import java.util.Optional;

/**
 * Field zones and targeting points for the 2026 REBUILT game on the RoboCon (modified) field.
 * Shooting at the hub from inside the neutral zone is illegal; there, fuel is funneled toward the
 * robot's own alliance corner instead (never across toward the opposing alliance).
 */
public class FieldConstants {
  // RoboCon (modified) field geometry, derived from the same sources vision uses so the zones
  // always agree with the pose frame: field size from the custom AprilTag map, hub centers from the
  // tape-measured VisionConstants values. Blue-origin frame (X along the field, Y across).
  private static final double FIELD_LENGTH_METERS = VisionConstants.aprilTagLayout.getFieldLength();
  private static final double FIELD_WIDTH_METERS = VisionConstants.aprilTagLayout.getFieldWidth();
  // Hub footprint is 47 in square, so each face sits 23.5 in from the hub center
  private static final double HUB_HALF_DEPTH_METERS = Units.inchesToMeters(23.5);
  // Bounds overshoot the field edges slightly for robustness
  private static final double EDGE_OVERSHOOT_METERS = Units.inchesToMeters(10);

  // Neutral zone: from the blue hub's far face to the red hub's near face (hub shots are illegal
  // inside it). Split in halves at the hubs' Y; "RIGHT" is the low-Y half as seen from the blue
  // driver station.
  private static final double NEUTRAL_MIN_X =
      VisionConstants.blueHubCenter.getX() + HUB_HALF_DEPTH_METERS;
  private static final double NEUTRAL_MAX_X =
      VisionConstants.redHubCenter.getX() - HUB_HALF_DEPTH_METERS;
  private static final double NEUTRAL_SPLIT_Y = VisionConstants.blueHubCenter.getY();

  public static final Region2d NEUTRAL_ZONE_RIGHT =
      new Region2d(
          new Translation2d[] {
            new Translation2d(NEUTRAL_MIN_X, -EDGE_OVERSHOOT_METERS), // bottom-left
            new Translation2d(NEUTRAL_MAX_X, -EDGE_OVERSHOOT_METERS), // bottom-right
            new Translation2d(NEUTRAL_MAX_X, NEUTRAL_SPLIT_Y), // top-right
            new Translation2d(NEUTRAL_MIN_X, NEUTRAL_SPLIT_Y) // top-left
          });

  public static final Region2d NEUTRAL_ZONE_LEFT =
      new Region2d(
          new Translation2d[] {
            new Translation2d(NEUTRAL_MIN_X, NEUTRAL_SPLIT_Y),
            new Translation2d(NEUTRAL_MAX_X, NEUTRAL_SPLIT_Y),
            new Translation2d(NEUTRAL_MAX_X, FIELD_WIDTH_METERS + EDGE_OVERSHOOT_METERS),
            new Translation2d(NEUTRAL_MIN_X, FIELD_WIDTH_METERS + EDGE_OVERSHOOT_METERS)
          });

  // Funnel aim points: just inside each corner of the robot's OWN alliance wall, so neutral-zone
  // fuel always travels back toward friendly territory. PLACEHOLDER inset — tune on the field.
  private static final double FUNNEL_CORNER_INSET_METERS = 1.0;
  public static final Translation2d BLUE_FUNNEL_TARGET_LOW_Y =
      new Translation2d(FUNNEL_CORNER_INSET_METERS, FUNNEL_CORNER_INSET_METERS);
  public static final Translation2d BLUE_FUNNEL_TARGET_HIGH_Y =
      new Translation2d(
          FUNNEL_CORNER_INSET_METERS, FIELD_WIDTH_METERS - FUNNEL_CORNER_INSET_METERS);
  public static final Translation2d RED_FUNNEL_TARGET_LOW_Y =
      new Translation2d(
          FIELD_LENGTH_METERS - FUNNEL_CORNER_INSET_METERS, FUNNEL_CORNER_INSET_METERS);
  public static final Translation2d RED_FUNNEL_TARGET_HIGH_Y =
      new Translation2d(
          FIELD_LENGTH_METERS - FUNNEL_CORNER_INSET_METERS,
          FIELD_WIDTH_METERS - FUNNEL_CORNER_INSET_METERS);

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
