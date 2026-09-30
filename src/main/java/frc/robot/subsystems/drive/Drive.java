// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LocalADStarAK;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  public static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;

  // maple-sim drivetrain physics configuration (simulation only). Motor models match PP_CONFIG.
  // Bumpers from CAD: 36.5 in fore-aft (X) by 33.5 in across the intake side (Y, front).
  // maple-sim drivetrain physics configuration (simulation only). Motor models match PP_CONFIG.
  // Bumpers from CAD: 36.5 in fore-aft (X) by 33.5 in across the intake side (Y, front).
  public static DriveTrainSimulationConfig getMapleSimConfig() {
    return DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
        .withBumperSize(Inches.of(36.5), Inches.of(33.5))
        .withCustomModuleTranslations(getModuleTranslations())
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(
            new SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getFalcon500(1),
                TunerConstants.FrontLeft.DriveMotorGearRatio,
                TunerConstants.FrontLeft.SteerMotorGearRatio,
                Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                Meters.of(TunerConstants.FrontLeft.WheelRadius),
                KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                WHEEL_COF));
  }

  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  // True once the pose estimator's heading is referenced to the field (see sampleHeadingAt)
  private boolean headingInitialized = false;

  // Vision snap (see requestVisionSnap): armed by a command, consumed by the next good measurement
  private boolean visionSnapPending = false;
  // False until a good heading-bearing (multitag) vision measurement has set the heading since the
  // last pose reset; that first one is applied as a hard heading reference (see
  // addVisionMeasurement)
  private boolean visionHeadingSeeded = false;
  private double visionSnapRequestTimestamp = 0.0;
  // Most recent PathPlanner setpoint (already alliance-flipped). When a path finishes this is its
  // end pose, which the auto Shoot drives to before firing (see getLastPathTargetPose)
  private Pose2d lastPathTargetPose = null;

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  // Teleports the maple-sim chassis when odometry is reset (no-op outside simulation)
  private final Consumer<Pose2d> resetSimulationPoseCallback;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO, pose -> {});
  }

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallback) {
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallback = resetSimulationPoseCallback;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // PathPlanner mirrors blue paths onto the red side using the field size, which defaults to the
    // stock 2026 field (16.54 m). The RoboCon field is shorter (~14.68 m), so without this every
    // red
    // pose lands ~1.86 m too far toward the red wall. Use the custom map's dimensions instead (the
    // map is rotationally symmetric, matching PathPlanner's default flip).
    FlippingUtil.fieldSizeX = VisionConstants.aprilTagLayout.getFieldLength();
    FlippingUtil.fieldSizeY = VisionConstants.aprilTagLayout.getFieldWidth();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
          lastPathTargetPose = targetPose;
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    Logger.recordOutput("Drive/HeadingInitialized", headingInitialized);
    Logger.recordOutput("Drive/VisionSnap/Pending", visionSnapPending);
    Logger.recordOutput("Drive/VisionHeadingSeed/Seeded", visionHeadingSeeded);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Measured field-relative translational velocity (from module states, rotated by the estimated
   * heading). Feeds the shoot-on-the-move lead compensation.
   */
  public Translation2d getFieldVelocity() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        .rotateBy(getRotation());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    resetSimulationPoseCallback.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    headingInitialized = true;
    visionSnapPending = false;
    // A reset (driver heading button, auto start) is only as accurate as the robot's placement, so
    // let the next good multitag measurement re-establish the heading precisely
    visionHeadingSeeded = false;
    lastPathTargetPose = null; // A new pose frame; old path setpoints no longer apply
  }

  /**
   * The last setpoint PathPlanner commanded (field frame, alliance-flipped) — after a path ends,
   * where that path meant the robot to stop. Empty if no path has run since the last pose reset.
   */
  public Optional<Pose2d> getLastPathTargetPose() {
    return Optional.ofNullable(lastPathTargetPose);
  }

  /**
   * Arms a vision snap: the next good vision measurement captured after this call is applied with a
   * near-zero X/Y std dev, so the pose estimator's position becomes the vision position (heading
   * stays with the gyro). Used after driving over the bump, where wheel slip corrupts odometry.
   * Odometry recorded since the camera frame is replayed on top, so camera latency is compensated.
   */
  public void requestVisionSnap() {
    visionSnapPending = true;
    visionSnapRequestTimestamp = Timer.getTimestamp();
  }

  /** Disarms a pending vision snap (no-op if it already happened). */
  public void cancelVisionSnap() {
    visionSnapPending = false;
  }

  /** True while a requested vision snap is still waiting for a good measurement. */
  public boolean isVisionSnapPending() {
    return visionSnapPending;
  }

  /** Adds a new timestamped vision measurement (or applies a pending vision snap). */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    if (visionSnapPending
        && timestampSeconds >= visionSnapRequestTimestamp // Frame captured after the request
        && visionMeasurementStdDevs.get(0, 0) <= VisionConstants.visionSnapMaxLinearStdDev) {
      visionSnapPending = false;
      poseEstimator
          .sampleAt(timestampSeconds)
          .ifPresent(
              before ->
                  Logger.recordOutput(
                      "Drive/VisionSnap/CorrectionMeters",
                      before.getTranslation().getDistance(visionRobotPoseMeters.getTranslation())));
      Logger.recordOutput("Drive/VisionSnap/SnapPose", visionRobotPoseMeters);
      visionMeasurementStdDevs =
          VecBuilder.fill(
              VisionConstants.visionSnapStdDev,
              VisionConstants.visionSnapStdDev,
              visionMeasurementStdDevs.get(2, 0));
    }
    // Heading seed: the first good heading-bearing measurement after boot or a pose reset SETS the
    // heading (tiny theta std dev) instead of nudging it. Without this the heading is only as good
    // as the gyro's power-on zero or where the robot pointed when the heading button was pressed —
    // and single-tag solves inherit any heading error as position error.
    double thetaStdDev = visionMeasurementStdDevs.get(2, 0);
    if (!visionHeadingSeeded
        && Double.isFinite(thetaStdDev)
        && thetaStdDev <= VisionConstants.visionHeadingSeedMaxStdDev) {
      visionHeadingSeeded = true;
      poseEstimator
          .sampleAt(timestampSeconds)
          .ifPresent(
              before ->
                  Logger.recordOutput(
                      "Drive/VisionHeadingSeed/CorrectionDeg",
                      visionRobotPoseMeters
                          .getRotation()
                          .minus(before.getRotation())
                          .getDegrees()));
      visionMeasurementStdDevs =
          VecBuilder.fill(
              visionMeasurementStdDevs.get(0, 0),
              visionMeasurementStdDevs.get(1, 0),
              VisionConstants.visionSnapStdDev);
    }
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    // A measurement that is allowed to correct heading anchors it to the field
    if (Double.isFinite(visionMeasurementStdDevs.get(2, 0))) {
      headingInitialized = true;
    }
  }

  /**
   * Returns the estimated field-relative heading at a past timestamp (for latency-compensated
   * single-tag vision solves), or empty if the heading has not yet been anchored to the field. At
   * boot the heading is just the gyro's power-on zero, so it only counts as anchored after a pose
   * reset (auto start / driver heading reset) or a heading-correcting multitag vision measurement.
   */
  public Optional<Rotation2d> sampleHeadingAt(double timestampSeconds) {
    if (!headingInitialized) {
      return Optional.empty();
    }
    return poseEstimator.sampleAt(timestampSeconds).map(Pose2d::getRotation);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
