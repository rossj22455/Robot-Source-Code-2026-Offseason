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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinates the PhotonVision cameras: feeds accepted pose observations to the drive pose
 * estimator (multitag solves correct X/Y/heading; single tags are solved from range + angle + gyro
 * heading and correct X/Y only), exposes target angles for game-piece funneling, and provides the
 * hub distance used by the shooter's RPM interpolation.
 *
 * <p>Fault tolerance: a disconnected camera reports {@code connected = false} and produces no
 * observations, so the pose estimator silently degrades to pure wheel odometry. No special-case
 * recovery is required.
 */
public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final HeadingSampler headingSampler;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private double lastAcceptedPoseTimestamp = Double.NEGATIVE_INFINITY;
  private final String[] lastRejectReasons;

  // Indexed to match the IO array order in RobotContainer
  private static final Transform3d[] robotToCameraTransforms = {
    robotToCamera0, robotToCamera1, robotToCamera2
  };

  public Vision(
      VisionConsumer consumer,
      Supplier<Pose2d> robotPoseSupplier,
      HeadingSampler headingSampler,
      VisionIO... io) {
    this.consumer = consumer;
    this.robotPoseSupplier = robotPoseSupplier;
    this.headingSampler = headingSampler;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    this.lastRejectReasons = new String[io.length];
    Arrays.fill(lastRejectReasons, "none");

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
          String rejectReason = getRejectReason(observation);

          // Add pose to log
          robotPoses.add(observation.pose());
          if (rejectReason != null) {
            robotPosesRejected.add(observation.pose());
            lastRejectReasons[cameraIndex] = "Multitag: " + rejectReason;
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
              VecBuilder.fill(
                  linearStdDev,
                  linearStdDev,
                  useVisionHeading ? angularStdDev : Double.POSITIVE_INFINITY));
          lastAcceptedPoseTimestamp = Timer.getTimestamp();
        }

        // Single-tag observations: solved from the camera-to-tag vector and the gyro heading (see
        // solveSingleTagPose). They correct X/Y only — heading stays with the gyro, which is far
        // more precise than a single tag's orientation.
        for (var observation : inputs[cameraIndex].singleTagObservations) {
          var tagPose = aprilTagLayout.getTagPose(observation.tagId());
          var heading = headingSampler.sample(observation.timestamp());
          String skipReason =
              tagPose.isEmpty()
                  ? "tag " + observation.tagId() + " not in field layout"
                  : heading.isEmpty()
                      ? "heading not set yet (press Circle or see 2+ tags)"
                      : cameraIndex >= robotToCameraTransforms.length
                          ? "no camera transform"
                          : !Double.isFinite(observation.cameraToTag().getNorm())
                              ? "non-finite tag distance"
                              : observation.cameraToTag().getNorm() > maxSingleTagDistance
                                  ? String.format(
                                      "tag %.2f m away > max %.2f m",
                                      observation.cameraToTag().getNorm(), maxSingleTagDistance)
                                  : null;
          if (skipReason != null) {
            lastRejectReasons[cameraIndex] = "Single tag skipped: " + skipReason;
            continue;
          }

          Pose2d pose =
              solveSingleTagPose(
                  tagPose.get(),
                  robotToCameraTransforms[cameraIndex],
                  heading.get(),
                  observation.cameraToTag());

          // Must be within the field boundaries
          String rejectReason = getOutOfFieldReason(pose.getX(), pose.getY());

          robotPoses.add(new Pose3d(pose));
          if (rejectReason != null) {
            robotPosesRejected.add(new Pose3d(pose));
            lastRejectReasons[cameraIndex] = "Single tag: " + rejectReason;
            continue;
          }
          robotPosesAccepted.add(new Pose3d(pose));

          double linearStdDev =
              linearStdDevBaseline * Math.pow(observation.cameraToTag().getNorm(), 2.0);
          if (cameraIndex < cameraStdDevFactors.length) {
            linearStdDev *= cameraStdDevFactors[cameraIndex];
          }
          consumer.accept(
              pose,
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, Double.POSITIVE_INFINITY));
          lastAcceptedPoseTimestamp = Timer.getTimestamp();
        }
      }

      // Log camera data. LastRejectReason keeps the most recent reason a pose was thrown out, so a
      // stream of rejections is diagnosable in AdvantageScope.
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/LastRejectReason", lastRejectReasons[cameraIndex]);
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

  /** Why a multitag observation must be rejected, or null if it is usable. */
  private static String getRejectReason(PoseObservation observation) {
    Pose3d pose = observation.pose();
    if (observation.tagCount() == 0) {
      return "no tags";
    }
    if (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) {
      return String.format("ambiguity %.2f > max %.2f", observation.ambiguity(), maxAmbiguity);
    }
    // Must be finite: NaN passes every </> comparison and would permanently corrupt the pose
    // estimator's Kalman state
    if (!Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getZ())
        || !Double.isFinite(observation.averageTagDistance())) {
      return "non-finite pose";
    }
    // Must have a realistic Z (the robot doesn't leave the floor). A consistently wrong Z usually
    // means the camera mount (robotToCamera height or pitch) is wrong.
    if (Math.abs(pose.getZ()) > maxZError) {
      return String.format(
          "Z %.2f m off the floor > max %.2f m (check camera height/pitch)",
          pose.getZ(), maxZError);
    }
    return getOutOfFieldReason(pose.getX(), pose.getY());
  }

  /** Why a pose is outside the field boundaries, or null if it is inside. */
  private static String getOutOfFieldReason(double x, double y) {
    double margin = fieldBoundsMarginMeters;
    if (x < -margin
        || x > aprilTagLayout.getFieldLength() + margin
        || y < -margin
        || y > aprilTagLayout.getFieldWidth() + margin) {
      return String.format(
          "outside field at (%.2f, %.2f); field is %.2f x %.2f m (+%.1f m margin)",
          x, y, aprilTagLayout.getFieldLength(), aprilTagLayout.getFieldWidth(), margin);
    }
    return null;
  }

  /**
   * Gyro-assisted single-tag pose solve (same idea as PhotonVision's PNP_DISTANCE_TRIG_SOLVE). Uses
   * only where the tag center is relative to the camera, plus the robot heading, so it cannot
   * suffer the orientation "flip" ambiguity of a single-tag PnP solve — the robust choice for a
   * single-camera robot.
   *
   * @param tagPose Field pose of the observed tag.
   * @param robotToCamera Camera mounting transform.
   * @param robotHeading Field-relative robot heading at the observation timestamp.
   * @param cameraToTagInCameraFrame Tag center in the camera frame (x forward, y left, z up).
   */
  public static Pose2d solveSingleTagPose(
      Pose3d tagPose,
      Transform3d robotToCamera,
      Rotation2d robotHeading,
      Translation3d cameraToTagInCameraFrame) {
    // Rotate the camera-to-tag vector into the robot frame (this also removes the camera's
    // mounting pitch, so the floor-plane component is correct)
    Translation2d cameraToTag =
        cameraToTagInCameraFrame.rotateBy(robotToCamera.getRotation()).toTranslation2d();

    // Walk back from the tag to the camera, then from the camera to the robot center, in the
    // field frame
    Translation2d fieldToCamera =
        tagPose.toPose2d().getTranslation().minus(cameraToTag.rotateBy(robotHeading));
    Translation2d fieldToRobot =
        fieldToCamera.minus(
            robotToCamera.getTranslation().toTranslation2d().rotateBy(robotHeading));
    return new Pose2d(fieldToRobot, robotHeading);
  }

  /** Returns the field-relative robot heading at a past timestamp, or empty if unknown. */
  @FunctionalInterface
  public static interface HeadingSampler {
    public Optional<Rotation2d> sample(double timestampSeconds);
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
