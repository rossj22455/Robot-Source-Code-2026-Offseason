// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Shoot-on-the-move solver: while the ball is in the air for time T, the robot's velocity is
 * carried with it, so the shot lands displaced by {@code v * T}. Compensation aims at a VIRTUAL
 * target displaced the opposite way ({@code hub - v * T}) and spins the drum for the virtual
 * distance. Both the heading and RPM setpoints then move smoothly and predictively as the robot
 * drives — the PIDs only clean up small errors instead of chasing.
 *
 * <p>Time of flight comes from the same vacuum-ballistics model the RPM table was fit with: the
 * ball leaves at {@code rpmMap(d) -> surface speed -> ball speed} on the fixed 64 degree hood, so
 * the time to cover horizontal distance d is {@code d / (ballSpeed * cos(hood))}. T depends on the
 * distance and the distance depends on T, so the solve iterates (converges in 2-3 passes).
 *
 * <p>Stationary robot degenerates to the plain hub aim (virtual target == hub). Physics caveat: the
 * 64 degree lob has a long flight time (~0.7-1.3 s across the table), so the lead grows fast with
 * speed — accuracy is best below ~1.5 m/s.
 */
public final class ShotOnMoveSolver {
  private static final int SOLVER_ITERATIONS = 3;
  private static final double COS_HOOD = Math.cos(Math.toRadians(ShooterConstants.HOOD_ANGLE_DEG));

  // Distance -> RPM, mirrored from the Shooter's table (built once; the map clamps to its edge
  // entries outside the tuned 1.7-4.3 m range, matching the Shooter's behavior)
  private static final InterpolatingDoubleTreeMap RPM_MAP = new InterpolatingDoubleTreeMap();

  static {
    for (double[] point : ShooterConstants.DISTANCE_TO_RPM_MAP) {
      RPM_MAP.put(point[0], point[1]);
    }
  }

  private ShotOnMoveSolver() {}

  /**
   * @param virtualTarget the displaced aim point the robot should treat as the hub
   * @param distanceMeters robot-to-virtual-target distance — feed this to the RPM map
   * @param heading field-relative bearing to the virtual target — the aim heading
   * @param timeOfFlightSecs predicted ball flight time to the hub
   * @param leadMeters how far the aim point was displaced (0 when stationary)
   */
  public record Solution(
      Translation2d virtualTarget,
      double distanceMeters,
      Rotation2d heading,
      double timeOfFlightSecs,
      double leadMeters) {}

  /**
   * Solves the moving shot.
   *
   * @param robotPosition field-relative robot position (pose estimate)
   * @param fieldVelocity field-relative robot translational velocity, m/s
   * @param target the real target (hub center)
   */
  public static Solution solve(
      Translation2d robotPosition, Translation2d fieldVelocity, Translation2d target) {
    Translation2d virtualTarget = target;
    double distance = robotPosition.getDistance(target);
    double timeOfFlight = 0.0;
    for (int i = 0; i < SOLVER_ITERATIONS; i++) {
      timeOfFlight = timeOfFlightSecs(distance);
      virtualTarget = target.minus(fieldVelocity.times(timeOfFlight));
      distance = robotPosition.getDistance(virtualTarget);
    }
    return new Solution(
        virtualTarget,
        distance,
        virtualTarget.minus(robotPosition).getAngle(),
        timeOfFlight,
        target.getDistance(virtualTarget));
  }

  /** Ball exit speed for a given shot distance, from the RPM table and drum geometry. */
  public static double ballSpeedMps(double distanceMeters) {
    return RPM_MAP.get(distanceMeters)
        / 60.0
        * 2.0
        * Math.PI
        * ShooterConstants.DRUM_RADIUS_METERS
        * ShooterConstants.SURFACE_TO_BALL_SPEED_RATIO;
  }

  /** Vacuum-ballistics flight time to cover the horizontal distance on the fixed hood angle. */
  public static double timeOfFlightSecs(double distanceMeters) {
    return distanceMeters / (ballSpeedMps(distanceMeters) * COS_HOOD);
  }
}
