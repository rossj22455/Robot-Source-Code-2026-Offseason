// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionConstants.CameraRole;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final CameraRole role;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera (must match the PhotonVision UI).
   * @param robotToCamera The 3D position of the camera relative to the robot.
   * @param role The role of the camera. GAMEPIECE cameras never produce pose observations.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera, CameraRole role) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.role = role;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // isConnected() is heartbeat-debounced by PhotonLib. It detects a dead coprocessor or NT
    // link; a stalled pipeline on a live coprocessor is covered separately by the timestamp
    // gating on target observations in Vision.
    inputs.connected = camera.isConnected();

    // Read new camera observations (non-blocking NetworkTables reads only)
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    List<SingleTagObservation> singleTagObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        var bestTarget = result.getBestTarget();
        inputs.latestTargetObservation =
            new TargetObservation(
                true,
                result.getTimestampSeconds(),
                Rotation2d.fromDegrees(bestTarget.getYaw()),
                Rotation2d.fromDegrees(bestTarget.getPitch()),
                bestTarget.getArea());
      } else {
        inputs.latestTargetObservation =
            new TargetObservation(
                false, result.getTimestampSeconds(), Rotation2d.kZero, Rotation2d.kZero, 0.0);
      }

      // Game-piece cameras never contribute to pose estimation
      if (role == CameraRole.GAMEPIECE) {
        continue;
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average distance over the tags actually used in the multitag solve (the
        // pipeline can detect tags the solver excluded; those contribute nothing to the pose)
        double totalTagDistance = 0.0;
        int usedTagCount = 0;
        for (var target : result.targets) {
          if (multitagResult.fiducialIDsUsed.contains((short) target.fiducialId)) {
            totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
            usedTagCount++;
          }
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation. An unmatched used-tag set yields an infinite average distance so the
        // observation is rejected by the finiteness filter rather than trusted at zero distance.
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                usedTagCount > 0
                    ? totalTagDistance / usedTagCount
                    : Double.POSITIVE_INFINITY)); // Average tag distance

      } else if (!result.targets.isEmpty()) { // Single tag result
        // Log the raw camera-to-tag vector rather than solving a pose here: the single-tag PnP
        // orientation is prone to ambiguity flips, so Vision solves the pose from the gyro heading
        // instead (and doing it there keeps it replayable).
        var target = result.targets.get(0);
        tagIds.add((short) target.fiducialId);
        singleTagObservations.add(
            new SingleTagObservation(
                result.getTimestampSeconds(),
                target.fiducialId,
                target.bestCameraToTarget.getTranslation(),
                target.poseAmbiguity));
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    inputs.singleTagObservations = singleTagObservations.toArray(new SingleTagObservation[0]);

    // Save tag IDs to inputs object
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
