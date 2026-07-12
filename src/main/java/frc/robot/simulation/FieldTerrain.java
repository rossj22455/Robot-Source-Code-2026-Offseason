// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Static height field of the 2026 REBUILT field terrain: the two hub ramps modeled as linear
 * slopes. Sampled per wheel-contact-point by {@link TerrainAwareSwerveDriveSimulation}, which turns
 * the geometry into real chassis dynamics (the ramp COLLIDERS are disabled in the arena so the
 * chassis can drive onto the zones — see RobotContainer).
 *
 * <p>Ramp geometry from maple-sim's 2026 REBUILT field map: each hub footprint is 47 in wide (X) by
 * 217 in long (Y); the central 47x47 in square is the hub itself (still a collider) and the two 85
 * in strips north/south of it are ramps rising toward the hub.
 */
public final class FieldTerrain {
  private static final double BLUE_HUB_X = 4.5974;
  private static final double RED_HUB_X = 11.938;
  private static final double HUB_Y = 4.034536;
  private static final double RAMP_HALF_WIDTH_X = Units.inchesToMeters(23.5);
  private static final double HUB_HALF_LENGTH_Y = Units.inchesToMeters(23.5);
  private static final double RAMP_OUTER_Y = Units.inchesToMeters(108.5);
  private static final double RAMP_LENGTH_METERS = RAMP_OUTER_Y - HUB_HALF_LENGTH_Y; // 85 in

  // PLACEHOLDER ramp rise over the 2.16 m ramp length (~5.3 deg slope). The tilt threshold is
  // 10 deg, so real-ish ramps do NOT trigger recovery; raise above ~0.38 m to force tilt events
  // when testing the recovery logic.
  public static final double RAMP_HEIGHT_METERS = 0.20;

  private FieldTerrain() {}

  /** Floor height at a field position, in meters (0 on the flat carpet). */
  public static double heightAt(double x, double y) {
    double dyFromHub = y - HUB_Y;
    for (double hubX : new double[] {BLUE_HUB_X, RED_HUB_X}) {
      if (onRampStrip(x, dyFromHub, hubX)) {
        return RAMP_HEIGHT_METERS * (RAMP_OUTER_Y - Math.abs(dyFromHub)) / RAMP_LENGTH_METERS;
      }
    }
    return 0.0;
  }

  /**
   * Terrain gradient (dh/dx, dh/dy) at a field position. Only the Y component is ever nonzero:
   * ramps rise toward each hub along Y. The lateral (X) ramp edges are treated as steps — a wheel
   * is either on or off the strip — which the per-wheel contact sampling converts into roll.
   */
  public static Translation2d gradientAt(double x, double y) {
    double dyFromHub = y - HUB_Y;
    for (double hubX : new double[] {BLUE_HUB_X, RED_HUB_X}) {
      if (onRampStrip(x, dyFromHub, hubX)) {
        return new Translation2d(
            0.0, -Math.signum(dyFromHub) * (RAMP_HEIGHT_METERS / RAMP_LENGTH_METERS));
      }
    }
    return Translation2d.kZero;
  }

  private static boolean onRampStrip(double x, double dyFromHub, double hubX) {
    return Math.abs(x - hubX) <= RAMP_HALF_WIDTH_X
        && Math.abs(dyFromHub) > HUB_HALF_LENGTH_Y
        && Math.abs(dyFromHub) <= RAMP_OUTER_Y;
  }
}
