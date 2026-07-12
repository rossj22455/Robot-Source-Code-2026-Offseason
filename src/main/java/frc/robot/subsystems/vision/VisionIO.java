// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(false, 0.0, Rotation2d.kZero, Rotation2d.kZero, 0.0);
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /**
   * Represents the angle to the current best target (AprilTag or game piece), not used for pose
   * estimation.
   */
  public static record TargetObservation(
      boolean hasTarget, double timestamp, Rotation2d tx, Rotation2d ty, double area) {}

  /** Represents a single robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

  public default void updateInputs(VisionIOInputs inputs) {}
}
