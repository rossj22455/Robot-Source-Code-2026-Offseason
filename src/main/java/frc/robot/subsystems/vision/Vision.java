// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinates the three-camera PhotonVision array: feeds accepted pose observations to the drive
 * pose estimator, exposes target angles for game-piece funneling, and provides the hub distance
 * used by the shooter's RPM interpolation.
 *
 * <p>Fault tolerance: a disconnected camera reports {@code connected = false} and produces no
 * observations, so the pose estimator silently degrades to pure wheel odometry. No special-case
 * recovery is required.
 */
public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private double lastAcceptedPoseTimestamp = Double.NEGATIVE_INFINITY;

  public Vision(VisionConsumer consumer, Supplier<Pose2d> robotPoseSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.robotPoseSupplier = robotPoseSupplier;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts (warnings, not errors — the robot degrades gracefully to
    // pure odometry when a camera fails)
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < disconnectedAlerts.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle (yaw) to the best target of the given camera, for game-piece funneling or
   * target alignment. Check {@link #hasTarget(int)} first.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  /**
   * Returns true if the given camera is connected and currently sees a target. Gated on the
   * observation timestamp so a stalled pipeline (with a live coprocessor heartbeat) cannot leave a
   * phantom target latched.
   */
  public boolean hasTarget(int cameraIndex) {
    var observation = inputs[cameraIndex].latestTargetObservation;
    return inputs[cameraIndex].connected
        && observation.hasTarget()
        && Timer.getTimestamp() - observation.timestamp() < targetObservationTimeoutSecs;
  }

  /**
   * Returns the distance in meters from the estimated robot pose to the alliance hub center. Feeds
   * the shooter's distance-to-RPM interpolation. Pose-based (rather than camera pitch geometry) so
   * it remains valid through momentary tag occlusion and benefits from all AprilTag cameras.
   */
  public double getHubDistanceMeters() {
    return robotPoseSupplier.get().getTranslation().getDistance(getHubCenter());
  }

  /**
   * Returns true when the pose estimate has been corrected by an accepted vision observation
   * recently. The shooter should gate automatic firing on this to avoid shooting on a stale pose.
   */
  public boolean hasHubPoseConfidence() {
    return Timer.getTimestamp() - lastAcceptedPoseTimestamp < hubPoseConfidenceTimeoutSecs;
  }

  /**
   * Returns the field-relative heading from the estimated robot pose to the alliance hub center —
   * the direction the shooter (robot +X) must face to score. Feeds auto-alignment and the shooter's
   * aim interlock.
   */
  public Rotation2d getHubHeading() {
    return getHubCenter().minus(robotPoseSupplier.get().getTranslation()).getAngle();
  }

  /** Alliance-aware hub center (the shoot-on-the-move solver leads this point). */
  public Translation2d getHubCenter() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? redHubCenter
        : blueHubCenter;
  }

  @Override
  public void periodic() {
    // Update inputs and process for logging/replay
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Log camera poses (robot pose + mounting transform) so they move with the robot in
    // AdvantageScope
    Pose3d robotPose = new Pose3d(robotPoseSupplier.get());
    Transform3d[] robotToCameraTransforms = {robotToCamera0, robotToCamera1, robotToCamera2};
    for (int i = 0; i < io.length && i < robotToCameraTransforms.length; i++) {
      Logger.recordOutput(
          "Vision/Camera" + i + "/Pose", robotPose.transformBy(robotToCameraTransforms[i]));
    }

    // Initialize summary logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations. GAMEPIECE cameras are skipped entirely (the IO layer also
      // never emits pose observations for them; this keeps replayed logs honest as well).
      if (cameraIndex < cameraRoles.length && cameraRoles[cameraIndex] == CameraRole.APRILTAG) {
        for (var observation : inputs[cameraIndex].poseObservations) {
          // Check whether to reject pose
          boolean rejectPose =
              observation.tagCount() == 0 // Must have at least one tag
                  || (observation.tagCount() == 1
                      && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                  || Math.abs(observation.pose().getZ())
                      > maxZError // Must have realistic Z coordinate

                  // Must be finite: NaN passes every </>/ comparison and would permanently
                  // corrupt the pose estimator's Kalman state
                  || !Double.isFinite(observation.pose().getX())
                  || !Double.isFinite(observation.pose().getY())
                  || !Double.isFinite(observation.pose().getZ())
                  || !Double.isFinite(observation.averageTagDistance())

                  // Must be within the field boundaries
                  || observation.pose().getX() < 0.0
                  || observation.pose().getX() > aprilTagLayout.getFieldLength()
                  || observation.pose().getY() < 0.0
                  || observation.pose().getY() > aprilTagLayout.getFieldWidth();

          // Add pose to log
          robotPoses.add(observation.pose());
          if (rejectPose) {
            robotPosesRejected.add(observation.pose());
            continue;
          }
          robotPosesAccepted.add(observation.pose());

          // Calculate standard deviations, scaled by distance and tag count
          double stdDevFactor =
              Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
          double linearStdDev = linearStdDevBaseline * stdDevFactor;
          double angularStdDev = angularStdDevBaseline * stdDevFactor;
          if (cameraIndex < cameraStdDevFactors.length) {
            linearStdDev *= cameraStdDevFactors[cameraIndex];
            angularStdDev *= cameraStdDevFactors[cameraIndex];
          }

          // Send vision observation to the pose estimator
          consumer.accept(
              observation.pose().toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
          lastAcceptedPoseTimestamp = Timer.getTimestamp();
        }
      }

      // Log camera data
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/HubDistanceMeters", getHubDistanceMeters());
    Logger.recordOutput("Vision/HasHubPoseConfidence", hasHubPoseConfidence());
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
