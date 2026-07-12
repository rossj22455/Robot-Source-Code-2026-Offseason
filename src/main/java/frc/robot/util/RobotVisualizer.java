// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Publishes robot-relative component poses ("Components", Pose3d[4]) for AdvantageScope's 3D
 * articulated-model view. Runs in ALL modes (it is a pure function of already-logged subsystem
 * state, so it renders identically in REAL, SIM, and REPLAY).
 *
 * <p>Convention: the AdvantageScope model config JSON normalizes each component mesh onto the grid
 * origin (zeroedPosition/zeroedRotations); the poses published here are mount + motion, carrying
 * each component from the origin onto the robot. See docs/SIMULATION.md.
 *
 * <p>Component order: [0] intake carriage, [1] shooter drum, [2] indexer roller, [3] kicker roller.
 */
public class RobotVisualizer {
  private static final double LOOP_PERIOD_SECS = 0.02;

  // --- Component mount positions in the ROBOT frame (x forward, y left, z up, meters) ---
  // CONVENTION (matches the AdvantageScope custom-assets guide): the model config JSON's
  // zeroedPosition/zeroedRotations only normalize the CAD mesh onto the grid origin; the poses
  // published here carry each component from the origin onto the robot. So each ZEROED_* value
  // is that component's mounted position measured from the robot center at floor level, at the
  // mechanism's homed/zero state. Fill from CAD or tune visually in the 3D view.
  private static final Translation3d ZEROED_INTAKE_CARRIAGE =
      new Translation3d(-0.240, 0.0, 0.28591); // Tuned visually in AdvantageScope
  private static final Translation3d ZEROED_DRUM =
      new Translation3d(0.0, 0.0, 0.0); // PLACEHOLDER — drum axis, from CAD
  private static final Translation3d ZEROED_INDEXER_ROLLER =
      new Translation3d(0.0, 0.0, 0.0); // PLACEHOLDER — from CAD
  private static final Translation3d ZEROED_KICKER_ROLLER =
      new Translation3d(0.0, 0.0, 0.0); // PLACEHOLDER — from CAD

  // Intake travel direction in the robot frame: down the 23-degree rack incline, extending out
  // the -X side of the robot toward the floor (both signs verified visually in AdvantageScope).
  private static final double EXTENSION_DIRECTION_X = -1.0;
  private static final Translation3d INTAKE_TRAVEL_UNIT_VECTOR =
      new Translation3d(
          EXTENSION_DIRECTION_X * Math.cos(IntakeConstants.RACK_ANGLE_RAD),
          0.0,
          -Math.sin(IntakeConstants.RACK_ANGLE_RAD));

  private final Supplier<Pose2d> robotPoseSupplier;
  private final DoubleSupplier intakePositionMeters;
  private final DoubleSupplier drumVelocityRpm;
  private final DoubleSupplier indexerVelocityRpm;
  private final DoubleSupplier kickerVelocityRpm;

  // Cosmetic spin angles, integrated from velocity (all spin about the robot Y axis — flip the
  // Rotation3d axis below if a mechanism's CAD axis differs)
  private double drumAngleRad = 0.0;
  private double indexerAngleRad = 0.0;
  private double kickerAngleRad = 0.0;

  public RobotVisualizer(
      Supplier<Pose2d> robotPoseSupplier,
      DoubleSupplier intakePositionMeters,
      DoubleSupplier drumVelocityRpm,
      DoubleSupplier indexerVelocityRpm,
      DoubleSupplier kickerVelocityRpm) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.intakePositionMeters = intakePositionMeters;
    this.drumVelocityRpm = drumVelocityRpm;
    this.indexerVelocityRpm = indexerVelocityRpm;
    this.kickerVelocityRpm = kickerVelocityRpm;
  }

  /** Called from Robot.robotPeriodic() every loop, after the scheduler runs. */
  public void update() {
    updatePredictedTrajectory();

    drumAngleRad += rpmToRadPerLoop(drumVelocityRpm.getAsDouble());
    indexerAngleRad += rpmToRadPerLoop(indexerVelocityRpm.getAsDouble());
    kickerAngleRad += rpmToRadPerLoop(kickerVelocityRpm.getAsDouble());

    Pose3d intakeCarriage =
        new Pose3d(
            ZEROED_INTAKE_CARRIAGE.plus(
                INTAKE_TRAVEL_UNIT_VECTOR.times(intakePositionMeters.getAsDouble())),
            Rotation3d.kZero);
    Pose3d drum = new Pose3d(ZEROED_DRUM, new Rotation3d(0.0, drumAngleRad, 0.0));
    Pose3d indexerRoller =
        new Pose3d(ZEROED_INDEXER_ROLLER, new Rotation3d(0.0, indexerAngleRad, 0.0));
    Pose3d kickerRoller =
        new Pose3d(ZEROED_KICKER_ROLLER, new Rotation3d(0.0, kickerAngleRad, 0.0));

    Logger.recordOutput(
        "Components", new Pose3d[] {intakeCarriage, drum, indexerRoller, kickerRoller});
  }

  private static double rpmToRadPerLoop(double rpm) {
    return rpm / 60.0 * 2.0 * Math.PI * LOOP_PERIOD_SECS;
  }

  /**
   * Publishes the predicted shot arc from the CURRENT drum RPM (vacuum ballistics at the fixed hood
   * angle) so the landing zone is visible live in AdvantageScope. Runs in every mode — on the real
   * robot it uses the estimated pose, so drivers see it during matches.
   */
  private void updatePredictedTrajectory() {
    double launchSpeedMps =
        Math.abs(drumVelocityRpm.getAsDouble())
            / 60.0
            * 2.0
            * Math.PI
            * ShooterConstants.DRUM_RADIUS_METERS
            * ShooterConstants.SURFACE_TO_BALL_SPEED_RATIO;

    // Only draw the parabola while the shooter is actually spinning
    if (launchSpeedMps < 1.0) {
      Logger.recordOutput("FieldSimulation/PredictedTrajectory", new Pose3d[0]);
      return;
    }

    Pose2d robotPose = robotPoseSupplier.get();
    double angleRad = Math.toRadians(ShooterConstants.HOOD_ANGLE_DEG);
    double vx = launchSpeedMps * Math.cos(angleRad);
    double vz = launchSpeedMps * Math.sin(angleRad);

    Rotation2d heading = robotPose.getRotation();
    Translation2d fieldStart =
        robotPose.getTranslation().plus(ShooterConstants.BALL_EXIT_OFFSET.rotateBy(heading));

    List<Pose3d> trajectoryPoints = new ArrayList<>();
    double z = ShooterConstants.BALL_EXIT_HEIGHT_METERS;
    for (double t = 0.0; z >= 0.0 && t < 3.0; t += 0.05) {
      z = ShooterConstants.BALL_EXIT_HEIGHT_METERS + (vz * t) - (4.905 * t * t);
      Translation2d currentXY = fieldStart.plus(new Translation2d(vx * t, heading));
      trajectoryPoints.add(
          new Pose3d(currentXY.getX(), currentXY.getY(), Math.max(z, 0.0), Rotation3d.kZero));
    }

    Logger.recordOutput(
        "FieldSimulation/PredictedTrajectory", trajectoryPoints.toArray(new Pose3d[0]));
  }
}
