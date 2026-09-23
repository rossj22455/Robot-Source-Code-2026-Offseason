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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Constants for the PhotonVision pose-estimation cameras. Only one camera is wired right now (a
 * single centerline, shooter-facing camera); the array-based implementation still supports adding
 * more — define another robotToCameraN and wire it in RobotContainer. All physical values are
 * PLACEHOLDERS to be measured and populated before use; filter thresholds and std-dev baselines are
 * the published AdvantageKit vision template defaults, to be tuned on the field.
 */
public class VisionConstants {
  // AprilTag layout. The competition uses a MODIFIED ("RoboCon") field, so tag poses are loaded
  // from a custom layout placed in the deploy directory rather than the stock 2026 field. The SAME
  // map must be uploaded to the PhotonVision coprocessor(s) separately (see the PhotonVision
  // multitag docs) — the coprocessor uses it for multitag solves; this copy is used for single-tag
  // solves and the field-boundary rejection in Vision. If the file is missing or unreadable it
  // falls back to the stock 2026 field and raises an alert, so a missing map fails loudly instead
  // of feeding silently-wrong poses to the estimator.
  public static final String CUSTOM_FIELD_FILENAME = "2026-robocon-welded-photonvision-wpilib.json";

  private static final Alert customFieldMissingAlert =
      new Alert(
          "Custom AprilTag layout '"
              + CUSTOM_FIELD_FILENAME
              + "' not found in deploy — using the stock 2026 field. Vision poses will be OFF on"
              + " the modified field.",
          AlertType.kWarning);

  public static AprilTagFieldLayout aprilTagLayout = loadAprilTagLayout();

  private static AprilTagFieldLayout loadAprilTagLayout() {
    Path path = Filesystem.getDeployDirectory().toPath().resolve(CUSTOM_FIELD_FILENAME);
    if (Files.exists(path)) {
      try {
        return new AprilTagFieldLayout(path);
      } catch (IOException e) {
        DriverStation.reportError(
            "Failed to parse " + CUSTOM_FIELD_FILENAME + ": " + e.getMessage(), false);
      }
    }
    // Missing or unparseable: fall back to the stock field and light the alert.
    // Competition-day note: the stock default is k2026RebuiltWelded; if you ever run the real
    // stock field on AndyMark tag mounts, that is k2026RebuiltAndymark.
    customFieldMissingAlert.set(true);
    return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

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

  // Hub center positions for shooter distance/aim. Modified ("RoboCon") field, measured
  // 2026-09-23 for the RED hub: 134 in (alliance wall -> front of hub) + 23.5 in (front -> center)
  // = 157.5 in from its alliance wall down the field, and ~159 in from the side wall to the hub
  // center (Y). The two hubs are 263 in apart (center to center). Placed in the blue-origin pose
  // frame: the blue hub sits 157.5 in from the blue wall (symmetric with the measured red hub) and
  // the red hub 263 in farther down-field. Anchored to the blue wall (X=0 in the pose frame) so it
  // does NOT depend on the JSON's declared field length being exact.
  // ASSUMES a symmetric field (blue hub also 157.5 in from its wall) with both hubs on the same Y.
  // If blue's wall distance differs, set blueHubCenter's X directly; if RoboCon has a single shared
  // hub, set redHubCenter = blueHubCenter.
  private static final double HUB_CENTER_FROM_ALLIANCE_WALL_METERS =
      Units.inchesToMeters(134.0 + 23.5); // 157.5 in
  private static final double HUB_TO_HUB_METERS = Units.inchesToMeters(263.0);
  private static final double HUB_CENTER_FROM_SIDE_WALL_METERS = Units.inchesToMeters(159.0);
  public static Translation2d blueHubCenter =
      new Translation2d(HUB_CENTER_FROM_ALLIANCE_WALL_METERS, HUB_CENTER_FROM_SIDE_WALL_METERS);
  public static Translation2d redHubCenter =
      new Translation2d(
          HUB_CENTER_FROM_ALLIANCE_WALL_METERS + HUB_TO_HUB_METERS,
          HUB_CENTER_FROM_SIDE_WALL_METERS);

  // How recently a vision pose must have been accepted for the hub distance to be trusted
  // (seconds) — PLACEHOLDER
  public static double hubPoseConfidenceTimeoutSecs = 1.0;

  // How recently a target observation must have arrived for hasTarget() to report true; guards
  // against a stalled camera pipeline whose coprocessor heartbeat stays alive — PLACEHOLDER
  public static double targetObservationTimeoutSecs = 0.5;
}
