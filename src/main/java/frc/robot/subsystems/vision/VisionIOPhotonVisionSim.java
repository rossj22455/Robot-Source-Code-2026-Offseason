// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionConstants.CameraRole;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * IO implementation for physics sim using PhotonVision's simulator.
 *
 * <p>APRILTAG cameras share a {@code VisionSystemSim} seeded with the field's AprilTag layout.
 *
 * <p>GAMEPIECE camera support is commented out below (no camera currently uses that role). When
 * re-enabled, GAMEPIECE cameras should register on a separate, target-free {@code VisionSystemSim}
 * so they report no targets in sim (matching real hardware, where PhotonLib cannot simulate ML
 * object-detection pipelines and the camera would otherwise "detect" AprilTags). To exercise
 * funneling logic in sim, add synthetic {@code VisionTargetSim} targets to the game-piece system
 * (documented stretch task).
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim aprilTagSim;
  // private static VisionSystemSim gamepieceSim;

  private final VisionSystemSim visionSim;
  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;
  // Only the camera that created the shared VisionSystemSim steps it: update() processes EVERY
  // camera on the system, so redundant per-camera calls multiply the (expensive) projection work
  // and were overrunning the 20ms loop
  private final boolean ownsVisionSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   * @param role The role of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, CameraRole role, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera, role);
    this.poseSupplier = poseSupplier;

    // Initialize the shared vision sim for this camera's role
    // if (role == CameraRole.APRILTAG) {
    ownsVisionSim = aprilTagSim == null;
    if (ownsVisionSim) {
      aprilTagSim = new VisionSystemSim("apriltag");
      aprilTagSim.addAprilTags(aprilTagLayout);
    }
    visionSim = aprilTagSim;
    // } else {
    //   ownsVisionSim = gamepieceSim == null;
    //   if (ownsVisionSim) {
    //     gamepieceSim = new VisionSystemSim("gamepiece");
    //   }
    //   visionSim = gamepieceSim;
    // }

    // Add sim camera. Modest resolution/FPS keeps the synchronous projection work well inside
    // the 20ms loop; real cameras run asynchronously on a coprocessor, so this pacing is also
    // more realistic than the (expensive) defaults.
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(90.0));
    cameraProperties.setFPS(20.0);
    cameraProperties.setAvgLatencyMs(30.0);
    cameraProperties.setLatencyStdDevMs(5.0);
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Step the shared sim exactly once per loop (see ownsVisionSim)
    if (ownsVisionSim) {
      visionSim.update(poseSupplier.get());
    }
    super.updateInputs(inputs);
  }
}
