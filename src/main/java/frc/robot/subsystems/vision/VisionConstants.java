// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the PhotonVision pose-estimation cameras. Only one camera is wired right now (a
 * single centerline, shooter-facing camera); the array-based implementation still supports adding
 * more — define another robotToCameraN and wire it in RobotContainer. All physical values are
 * PLACEHOLDERS to be measured and populated before use; filter thresholds and std-dev baselines are
 * the published AdvantageKit vision template defaults, to be tuned on the field.
 */
public class VisionConstants {
  // AprilTag layout — 2026 REBUILT welded field (kDefaultField == k2026RebuiltWelded).
  // Competition-day check: switch to k2026RebuiltAndymark if the event uses AndyMark tag mounts.
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /** How each camera is used. GAMEPIECE cameras never contribute to pose estimation. */
  public static enum CameraRole {
    APRILTAG,
    GAMEPIECE
  }

  // Camera names — PLACEHOLDER: must exactly match the names configured in the PhotonVision web UI
  public static String camera0Name = "camera_0"; // Middle, shooter-facing (only camera wired now)
  public static String camera1Name = "camera_1"; // Left Side (example mount — not wired yet)
  public static String camera2Name = "camera_2"; // Right Side (example mount — not wired yet)

  // Camera roles, indexed to match the IO array order in RobotContainer. Only index 0 is wired
  // right now. GAMEPIECE role is unused for now (see VisionIOPhotonVisionSim's gamepieceSim, which
  // is commented out to match).
  public static CameraRole[] cameraRoles =
      new CameraRole[] {CameraRole.APRILTAG, CameraRole.APRILTAG, CameraRole.APRILTAG};

  // Robot-to-camera transforms — PLACEHOLDER: measure from robot center (x forward, y left, z up).
  // Only camera0 is wired right now (see RobotContainer): a single camera on the centerline (y=0),
  // shooter-facing, pitched up 15 deg. NOTE: -15 deg here means tilted UP 15 deg (WPILib pitch is
  // positive-down); flip the sign if the camera is physically tilted down. camera1/camera2 below
  // are example side mounts kept for when more cameras are added — they are not instantiated yet.
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(14.172905),
          0,
          Units.inchesToMeters(12.381851),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-15), Units.degreesToRadians(0)));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(7.342),
          Units.inchesToMeters(12.580287),
          Units.inchesToMeters(8.956792),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(135)));
  public static Transform3d robotToCamera2 =
      new Transform3d(
          Units.inchesToMeters(14.172905),
          0,
          Units.inchesToMeters(12.381851),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(0)));

  // Pose-observation rejection thresholds (AdvantageKit vision template defaults — tune on field)
  public static double maxAmbiguity = 0.3; // Single-tag ambiguity cutoff
  public static double maxZError = 0.75; // Meters; the robot should not leave the floor

  // Standard deviation baselines for 1 tag at 1 meter distance (AdvantageKit template defaults;
  // adjusted automatically based on distance and number of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Per-camera trust multipliers (>= 1 means less trusted) — PLACEHOLDER
  public static double[] cameraStdDevFactors = new double[] {1.0, 1.0, 1.0};

  // Hub center positions for shooter distance calculation. Values match the 2026 REBUILT field
  // (same coordinates maple-sim's RebuiltHub uses); verify against the official field drawings
  // before competition.
  public static Translation2d blueHubCenter = new Translation2d(4.5974, 4.034536);
  public static Translation2d redHubCenter = new Translation2d(11.938, 4.034536);

  // How recently a vision pose must have been accepted for the hub distance to be trusted
  // (seconds) — PLACEHOLDER
  public static double hubPoseConfidenceTimeoutSecs = 1.0;

  // How recently a target observation must have arrived for hasTarget() to report true; guards
  // against a stalled camera pipeline whose coprocessor heartbeat stays alive — PLACEHOLDER
  public static double targetObservationTimeoutSecs = 0.5;
}
