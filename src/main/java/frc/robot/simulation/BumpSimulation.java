// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/**
 * Synthetic terrain for the simulated gyro: maple-sim's physics are 2D, so driving over the field
 * ramps cannot tilt the chassis — instead this models the ramp slopes as a height field and injects
 * the resulting pitch/roll into {@code GyroIOSim}. This is what makes the tilt-detection and
 * tilt-recovery logic testable in simulation (the ramp COLLIDERS must be disabled in the arena so
 * the chassis can actually drive onto the zones — see RobotContainer).
 *
 * <p>Ramp geometry from maple-sim's 2026 REBUILT field map: each hub footprint is 47 in wide (X) by
 * 217 in long (Y); the central 47x47 in square is the hub itself (still a collider) and the two 85
 * in strips north/south of it are ramps rising toward the hub.
 */
public class BumpSimulation {
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
  private static final double RAMP_HEIGHT_METERS = 0.20;

  private Rotation2d pitch = Rotation2d.kZero;
  private Rotation2d roll = Rotation2d.kZero;
  private double heightMeters = 0.0;

  /** Called each sim loop with the ground-truth chassis pose. */
  public void update(Pose2d pose) {
    // Terrain gradient and height at the robot position (only the Y gradient is ever nonzero:
    // ramps rise toward each hub along Y)
    double gradientY = 0.0;
    heightMeters = 0.0;
    double dyFromHub = pose.getY() - HUB_Y;
    for (double hubX : new double[] {BLUE_HUB_X, RED_HUB_X}) {
      boolean onRampStrip =
          Math.abs(pose.getX() - hubX) <= RAMP_HALF_WIDTH_X
              && Math.abs(dyFromHub) > HUB_HALF_LENGTH_Y
              && Math.abs(dyFromHub) <= RAMP_OUTER_Y;
      if (onRampStrip) {
        gradientY = -Math.signum(dyFromHub) * (RAMP_HEIGHT_METERS / RAMP_LENGTH_METERS);
        heightMeters =
            RAMP_HEIGHT_METERS * (RAMP_OUTER_Y - Math.abs(dyFromHub)) / RAMP_LENGTH_METERS;
      }
    }

    // Project the slope onto the robot frame: pitch is nose-up-positive when climbing, roll is
    // left-side-up positive. (The tilt monitor thresholds on magnitude, so exact sign
    // conventions only matter for display.)
    double cos = pose.getRotation().getCos();
    double sin = pose.getRotation().getSin();
    pitch = Rotation2d.fromRadians(Math.atan(gradientY * sin));
    roll = Rotation2d.fromRadians(Math.atan(gradientY * cos));

    Logger.recordOutput("FieldSimulation/Terrain/PitchDeg", pitch.getDegrees());
    Logger.recordOutput("FieldSimulation/Terrain/RollDeg", roll.getDegrees());
    Logger.recordOutput("FieldSimulation/Terrain/OnRamp", gradientY != 0.0);
  }

  public Rotation2d getPitch() {
    return pitch;
  }

  public Rotation2d getRoll() {
    return roll;
  }

  /** Terrain height under the robot center (for the 3D bobbing robot pose). */
  public double getHeightMeters() {
    return heightMeters;
  }
}
