// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.drive.TiltConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.littletonrobotics.junction.Logger;

/**
 * Fault-tolerant autonomous path following with dynamic tilt recovery.
 *
 * <p>Every path segment runs under a continuous Pigeon 2 pitch/roll monitor (see {@link
 * Drive#isTilted()}). When a tilt anomaly is detected the current path command is immediately
 * interrupted, the robot performs a brief robot-relative reverse maneuver to dislodge, and
 * PathPlanner's on-the-fly pathfinding routes the robot back to the interrupted segment's end pose
 * (the next intended waypoint) so the routine safely resumes with the following segment.
 */
public class TiltRecoveryCommands {
  private static final PathConstraints RECOVERY_CONSTRAINTS =
      new PathConstraints(
          RECOVERY_MAX_VELOCITY_METERS_PER_SEC,
          RECOVERY_MAX_ACCELERATION_METERS_PER_SEC_SQ,
          Units.degreesToRadians(RECOVERY_MAX_ANGULAR_VELOCITY_DEG_PER_SEC),
          Units.degreesToRadians(RECOVERY_MAX_ANGULAR_ACCELERATION_DEG_PER_SEC_SQ));

  private static final Alert pathLoadFailedAlert =
      new Alert("Failed to load a PathPlanner path for a recoverable auto.", AlertType.kError);

  private TiltRecoveryCommands() {}

  /**
   * Builds a full auto routine from PathPlanner path files where every segment is wrapped with tilt
   * monitoring and recovery. If any path fails to load, an alert is raised and an empty command is
   * returned (the robot will not move rather than running a partial routine).
   *
   * @param drive The drive subsystem.
   * @param pathNames PathPlanner path file names, in execution order.
   */
  public static Command recoverableAuto(Drive drive, String... pathNames) {
    Command[] segments = new Command[pathNames.length];
    for (int i = 0; i < pathNames.length; i++) {
      try {
        segments[i] = followPathWithRecovery(drive, PathPlannerPath.fromPathFile(pathNames[i]));
      } catch (Exception e) {
        pathLoadFailedAlert.setText("Failed to load PathPlanner path: " + pathNames[i]);
        pathLoadFailedAlert.set(true);
        return Commands.none();
      }
    }
    return Commands.sequence(segments).withName("RecoverableAuto");
  }

  /**
   * Follows a single PathPlanner path with tilt monitoring. On a tilt anomaly the path command is
   * interrupted and the recovery sequence routes to the path's end pose (the next intended
   * waypoint), after which the outer sequence continues with the next segment.
   */
  public static Command followPathWithRecovery(Drive drive, PathPlannerPath path) {
    List<Pose2d> pathPoses = path.getPathPoses();
    // Blue-alliance-relative resume waypoint: the segment's final position with its goal rotation
    Pose2d resumePose =
        new Pose2d(
            pathPoses.get(pathPoses.size() - 1).getTranslation(),
            path.getGoalEndState().rotation());
    return withTiltRecovery(drive, AutoBuilder.followPath(path), resumePose);
  }

  /**
   * Wraps any autonomous segment with continuous tilt monitoring and the dynamic recovery sequence.
   *
   * @param drive The drive subsystem.
   * @param segment The command to monitor (interrupted immediately on tilt).
   * @param resumePoseBlue Blue-alliance-relative pose to pathfind back to after recovering (flipped
   *     automatically on the red alliance).
   */
  public static Command withTiltRecovery(Drive drive, Command segment, Pose2d resumePoseBlue) {
    // Latches whether the segment ended because of a tilt (reset at the start of every run so
    // the command factory is safe to schedule repeatedly)
    AtomicBoolean tiltDetected = new AtomicBoolean(false);
    return Commands.sequence(
            Commands.runOnce(() -> tiltDetected.set(false)),
            // Race: the segment is interrupted the moment a tilt anomaly is detected
            segment.raceWith(
                Commands.waitUntil(drive::isTilted)
                    .andThen(
                        Commands.runOnce(
                            () -> {
                              tiltDetected.set(true);
                              Logger.recordOutput(
                                  "Auto/TiltRecoveryTriggeredPose", drive.getPose());
                            }))),
            // Recover and re-route only if a tilt actually occurred
            Commands.either(
                recoverySequence(drive, resumePoseBlue), Commands.none(), tiltDetected::get))
        .withName("TiltMonitored");
  }

  /**
   * The recovery maneuver: brief robot-relative reverse to dislodge, then on-the-fly pathfinding
   * back to the given blue-alliance-relative waypoint. Note: the recovery itself is not
   * re-monitored for tilt; if the robot is still tilted after recovery the next monitored segment
   * triggers again.
   */
  public static Command recoverySequence(Drive drive, Pose2d resumePoseBlue) {
    return Commands.sequence(
            // Brief reverse maneuver to dislodge (robot-relative)
            drive
                .run(
                    () ->
                        drive.runVelocity(
                            new ChassisSpeeds(-RECOVERY_REVERSE_SPEED_METERS_PER_SEC, 0.0, 0.0)))
                .withTimeout(RECOVERY_REVERSE_DURATION_SECS),
            Commands.runOnce(drive::stop, drive),
            // On-the-fly generation back to the next intended waypoint
            AutoBuilder.pathfindToPoseFlipped(resumePoseBlue, RECOVERY_CONSTRAINTS))
        .withName("TiltRecovery");
  }
}
