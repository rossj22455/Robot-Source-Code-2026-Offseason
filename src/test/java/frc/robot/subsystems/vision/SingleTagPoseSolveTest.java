package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

/** Verifies the gyro-assisted single-tag solve against synthetic observations on the real map. */
class SingleTagPoseSolveTest {
  @Test
  void customFieldLayoutLoads() {
    // The RoboCon map (not the stock field fallback) must be the one in use
    assertEquals(32, VisionConstants.aprilTagLayout.getTags().size());
    assertTrue(VisionConstants.aprilTagLayout.getTagPose(3).isPresent());
  }

  @Test
  void recoversRobotPoseFromCameraToTagVector() {
    // Robot positions in front of the blue hub, facing roughly toward it, at typical shot ranges
    Pose2d[] robotPoses = {
      new Pose2d(1.5, 4.0, Rotation2d.fromDegrees(0.0)),
      new Pose2d(1.0, 3.0, Rotation2d.fromDegrees(20.0)),
      new Pose2d(0.8, 5.2, Rotation2d.fromDegrees(-25.0)),
    };
    int[] tagIds = {25, 26};

    for (Pose2d robotPose : robotPoses) {
      for (int tagId : tagIds) {
        Pose3d tagPose = VisionConstants.aprilTagLayout.getTagPose(tagId).orElseThrow();
        Pose3d cameraPose = new Pose3d(robotPose).transformBy(VisionConstants.robotToCamera0);
        Translation3d cameraToTag = tagPose.relativeTo(cameraPose).getTranslation();

        Pose2d solved =
            Vision.solveSingleTagPose(
                tagPose, VisionConstants.robotToCamera0, robotPose.getRotation(), cameraToTag);

        assertEquals(robotPose.getX(), solved.getX(), 1e-6, "x for tag " + tagId);
        assertEquals(robotPose.getY(), solved.getY(), 1e-6, "y for tag " + tagId);
        assertEquals(robotPose.getRotation().getRadians(), solved.getRotation().getRadians(), 1e-9);
      }
    }
  }
}
