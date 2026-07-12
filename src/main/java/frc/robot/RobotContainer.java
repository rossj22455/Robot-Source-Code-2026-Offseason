// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TiltRecoveryCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.SimulationManager;
import frc.robot.simulation.TerrainAwareSwerveDriveSimulation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.indexer.IndexerIOTalonFXSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeLinearIO;
import frc.robot.subsystems.intake.IntakeLinearIOSparkMax;
import frc.robot.subsystems.intake.IntakeLinearIOSparkMaxSim;
import frc.robot.subsystems.intake.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeRollerIOSparkMax;
import frc.robot.subsystems.intake.IntakeRollerIOSparkMaxSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterDrumIO;
import frc.robot.subsystems.shooter.ShooterDrumIOTalonFX;
import frc.robot.subsystems.shooter.ShooterDrumIOTalonFXSim;
import frc.robot.subsystems.shooter.ShooterKickerIO;
import frc.robot.subsystems.shooter.ShooterKickerIOSparkMax;
import frc.robot.subsystems.shooter.ShooterKickerIOSparkMaxSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.RobotVisualizer;
import frc.robot.util.ShotOnMoveSolver;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;

  // maple-sim field/game-piece simulation (SIM mode only, null otherwise)
  private TerrainAwareSwerveDriveSimulation driveSimulation = null;
  private SimulationManager simulationManager = null;

  // 3D component pose publisher for AdvantageScope (all modes, including replay)
  private final RobotVisualizer robotVisualizer;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVision(camera0Name, robotToCamera0, cameraRoles[0]),
                new VisionIOPhotonVision(camera1Name, robotToCamera1, cameraRoles[1]),
                new VisionIOPhotonVision(camera2Name, robotToCamera2, cameraRoles[2]));

        intake = new Intake(new IntakeLinearIOSparkMax(), new IntakeRollerIOSparkMax());
        shooter =
            new Shooter(
                new ShooterDrumIOTalonFX(),
                new ShooterKickerIOSparkMax(),
                this::getHubShotDistance,
                vision::hasHubPoseConfidence,
                drive::getRotation,
                this::getTargetHeading,
                this::isFunneling);
        indexer = new Indexer(new IndexerIOTalonFX(), shooter::isReadyToShoot);
        break;

      case SIM:
        // Sim robot: maple-sim physics arena drives the real device code through Phoenix/REV
        // sim states, so onboard control loops and device configs are exercised on desktop.
        // Ramp COLLIDERS are disabled so the chassis can drive onto the ramp zones; the
        // terrain-aware drive simulation models the ramps as real 3D chassis dynamics there
        // (slope forces, per-wheel loads, gyro pitch/roll — tilt logic testable).
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));

        // Spawn on the open field — (0,0) is the field corner and puts the chassis inside the
        // wall colliders (only useful when aligning component meshes to the grid origin)
        driveSimulation =
            new TerrainAwareSwerveDriveSimulation(
                Drive.getMapleSimConfig(), new Pose2d(3, 3, Rotation2d.kZero));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        drive =
            new Drive(
                new GyroIOSim(
                    driveSimulation.getGyroSimulation(),
                    driveSimulation::getPitch,
                    driveSimulation::getRoll),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        // Cameras observe the simulation's ground-truth pose (NOT odometry) — this is what makes
        // the vision-confidence firing gate testable in sim
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVisionSim(
                    camera0Name,
                    robotToCamera0,
                    cameraRoles[0],
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name,
                    robotToCamera1,
                    cameraRoles[1],
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera2Name,
                    robotToCamera2,
                    cameraRoles[2],
                    driveSimulation::getSimulatedDriveTrainPose));

        intake = new Intake(new IntakeLinearIOSparkMaxSim(), new IntakeRollerIOSparkMaxSim());
        shooter =
            new Shooter(
                new ShooterDrumIOTalonFXSim(),
                new ShooterKickerIOSparkMaxSim(),
                this::getHubShotDistance,
                vision::hasHubPoseConfidence,
                drive::getRotation,
                this::getTargetHeading,
                this::isFunneling);
        indexer = new Indexer(new IndexerIOTalonFXSim(), shooter::isReadyToShoot);

        // Game-piece lifecycle: field fuel -> intake -> launched shots (reads subsystem state
        // through suppliers only)
        simulationManager =
            new SimulationManager(
                driveSimulation,
                intake::getLinearPositionMeters,
                intake::getRollerAppliedVolts,
                shooter::getDrumVelocityRpm,
                shooter::getKickerAppliedVolts,
                indexer::getAppliedVolts,
                shooter::simNotifyBallFired);

        // Automated self-test: with SIM_TEST=1 the robot enables itself (see Robot) and runs a
        // scripted homing -> intake -> shoot sequence, printing [SIMTEST] markers for CI/log
        // inspection
        if ("1".equals(System.getenv("SIM_TEST"))) {
          configureSimTestSequence();
        }
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});

        intake = new Intake(new IntakeLinearIO() {}, new IntakeRollerIO() {});
        shooter =
            new Shooter(
                new ShooterDrumIO() {},
                new ShooterKickerIO() {},
                this::getHubShotDistance,
                vision::hasHubPoseConfidence,
                drive::getRotation,
                this::getTargetHeading,
                this::isFunneling);
        indexer = new Indexer(new IndexerIO() {}, shooter::isReadyToShoot);
        break;
    }

    // 3D component visualization (pure output from logged state, so it also renders in replay)
    robotVisualizer =
        new RobotVisualizer(
            drive::getPose,
            intake::getLinearPositionMeters,
            shooter::getDrumVelocityRpm,
            indexer::getVelocityRpm,
            shooter::getKickerVelocityRpm);

    // Named commands for PathPlanner autos — MUST be registered before building the chooser.
    // "Shoot" aims (sticks read zero in auto, so the robot rotates in place onto the target) and
    // fires volleys until the timeout. "IntakeRun" deploys + runs rollers and is meant for PP
    // event zones or deadline groups (it never ends on its own).
    NamedCommands.registerCommand("Shoot", aimAndShootCommand().withTimeout(4.0));
    NamedCommands.registerCommand("IntakeRun", intake.intakeCommand());
    NamedCommands.registerCommand("IntakeStop", Commands.runOnce(intake::stopRollers, intake));
    NamedCommands.registerCommand("IntakeRetract", intake.retractCommand());
    // Pre-spin while driving to the shooting pose (latched; the next Shoot's end, or disabling,
    // stops it) — the drum reaches the mapped RPM during travel so Shoot only needs aim+staging
    NamedCommands.registerCommand("SpinUp", Commands.runOnce(shooter::startSpinUp, shooter));
    NamedCommands.registerCommand("ShooterStop", Commands.runOnce(shooter::stopShooter, shooter));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Example fault-tolerant auto: every segment monitored for tilt with dynamic OTF recovery.
    // Add real routines by listing their PathPlanner path files in execution order.
    autoChooser.addOption(
        "Example Path (Tilt Recovery)",
        TiltRecoveryCommands.recoverableAuto(drive, "Example Path"));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .cross()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.square().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .circle()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // ---- Mechanism bindings (PLACEHOLDER layout — adjust to driver preference) ----

    // Deploy the intake (stays out afterward) and run the rollers while held
    controller.L2().whileTrue(intake.intakeCommand());

    controller.L1().onTrue(intake.retractCommand());

    // Full firing sequence while held: heading auto-locks onto the hub (driver keeps translation
    // on the sticks) while the drum spins to the vision-mapped RPM; the kicker and indexer feed
    // automatically the moment isReadyToShoot() is satisfied (at-speed + fresh vision + aimed —
    // re-evaluated every loop inside the subsystems)
    controller.R2().whileTrue(aimAndShootCommand());

    // Unjam while held: reverse the indexer belt and kicker together (always permitted)
    controller.R1().whileTrue(unjamCommand());
  }

  /**
   * Vision auto-alignment + firing: locks the robot heading onto the current target (hub, or the
   * alliance corner while funneling) while running the full firing sequence. The aim interlock
   * inside {@link Shooter#isReadyToShoot()} holds feeding until the heading converges, so volleys
   * only leave when genuinely on target. The intake sweeps in and shutters to herd balls for the
   * duration, returning to its extended posture afterward.
   */
  // While aiming+shooting, cap translation to this fraction of max speed (~1.1 m/s of the
  // 4.58 m/s max). The 64 deg lob flies ~1 s, so lead error scales with speed x time-of-flight
  // error — the shoot-on-the-move compensation is sim-validated up to ~1.5 m/s, and this keeps
  // the driver inside that envelope while still able to reposition.
  private static final double SHOOT_ON_MOVE_SPEED_SCALAR = 0.25;

  private Command aimAndShootCommand() {
    return Commands.parallel(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                this::getTargetHeading,
                SHOOT_ON_MOVE_SPEED_SCALAR),
            shootCommand())
        .withName("AimAndShoot");
  }

  /**
   * Field-relative bearing the shooter should face: the alliance corner while inside the neutral
   * zone (hub shots are illegal there — fuel funnels back to friendly territory), otherwise the
   * velocity-compensated hub bearing (shoot-on-the-move: aim leads the hub by robot velocity x
   * time-of-flight; degenerates to the plain hub bearing when stationary). Pose-based, so it keeps
   * working on pure odometry when vision drops out.
   */
  private Rotation2d getTargetHeading() {
    var robotPosition = drive.getPose().getTranslation();
    return FieldConstants.getFunnelTarget(robotPosition)
        .map(target -> target.minus(robotPosition).getAngle())
        .orElseGet(() -> getHubShotSolution().heading());
  }

  /**
   * Shoot-on-the-move solution for the hub: the virtual target the robot should aim at and range
   * against, leading the real hub by the robot's velocity over the ball's flight time. Recomputed
   * on demand from the latest pose estimate (cheap — a 3-iteration closed-form solve).
   */
  private ShotOnMoveSolver.Solution getHubShotSolution() {
    var solution =
        ShotOnMoveSolver.solve(
            drive.getPose().getTranslation(), drive.getFieldVelocity(), vision.getHubCenter());
    Logger.recordOutput(
        "Shooter/ShotOnMove/VirtualTarget", new Pose2d(solution.virtualTarget(), Rotation2d.kZero));
    Logger.recordOutput("Shooter/ShotOnMove/LeadMeters", solution.leadMeters());
    Logger.recordOutput("Shooter/ShotOnMove/TimeOfFlightSecs", solution.timeOfFlightSecs());
    return solution;
  }

  /** Range for the drum RPM map: distance to the virtual (velocity-compensated) hub target. */
  private double getHubShotDistance() {
    return getHubShotSolution().distanceMeters();
  }

  /** True while the robot is inside the neutral zone (shooter lobs toward the alliance corner). */
  private boolean isFunneling() {
    return FieldConstants.getFunnelTarget(drive.getPose().getTranslation()).isPresent();
  }

  /**
   * Declares the robot's current physical facing relative to the driver, fixing the pose
   * estimator's heading so field-relative driving is immediately correct. Alliance-aware: driver
   * forward means away from your own alliance wall. Pass {@code kZero} when the shooter (robot
   * front) points away from the driver, {@code k180deg} when the intake does.
   */
  // private Command declareHeadingCommand(Rotation2d robotFacingRelativeToDriverForward) {
  //   return Commands.runOnce(
  //           () -> {
  //             boolean isRed =
  //                 DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
  //                     == DriverStation.Alliance.Red;
  //             Rotation2d driverForward = isRed ? Rotation2d.k180deg : Rotation2d.kZero;
  //             drive.setPose(
  //                 new Pose2d(
  //                     drive.getPose().getTranslation(),
  //                     driverForward.plus(robotFacingRelativeToDriverForward)));
  //           },
  //           drive)
  //       .ignoringDisable(true);
  // }

  private Command shootCommand() {
    return Commands.runEnd(
            () -> {
              shooter.startSpinUp();
              shooter.runKickerFeed(); // No-op until isReadyToShoot()
              indexer.feed(); // Blocked by the interlock until isReadyToShoot()
              intake.startAggregating(); // Sweep in + shutter to herd balls (idempotent)
            },
            () -> {
              shooter.stopShooter();
              shooter.stopKicker();
              indexer.stop();
              intake.stopAggregating(); // Back to the resting EXTENDED posture
            },
            shooter,
            indexer,
            intake)
        .withName("Shoot");
  }

  /** Reverses the indexer and kicker together to clear a jam. */
  private Command unjamCommand() {
    return Commands.runEnd(
            () -> {
              shooter.runKickerReverse();
              indexer.reverse();
            },
            () -> {
              shooter.stopKicker();
              indexer.stop();
            },
            shooter,
            indexer)
        .withName("Unjam");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Scripted simulation self-test (SIM_TEST=1 only): after teleop enable, waits out homing, then
   * exercises the intake and the full firing chain, printing [SIMTEST] state markers.
   */
  private void configureSimTestSequence() {
    Command testSequence =
        Commands.sequence(
                Commands.print("[SIMTEST] enabled - waiting for intake homing"),
                Commands.waitSeconds(5.0),
                Commands.runOnce(
                    () ->
                        System.out.printf(
                            "[SIMTEST] homing: homed=%b pos=%.4f m%n",
                            intake.isHomed(), intake.getLinearPositionMeters())),
                Commands.runOnce(() -> simulationManager.preloadFuel(2)),
                Commands.print("[SIMTEST] intake deploy test (driving forward)"),
                Commands.deadline(
                    Commands.sequence(
                        Commands.waitSeconds(3.5),
                        Commands.runOnce(
                            () ->
                                System.out.printf(
                                    "[SIMTEST] deployed: pos=%.4f m atGoal=%b%n",
                                    intake.getLinearPositionMeters(), intake.isAtGoal()))),
                    intake.intakeCommand(),
                    // Drive intake-first (the intake is on the -X side of the robot)
                    DriveCommands.joystickDrive(drive, () -> -0.3, () -> 0.0, () -> 0.0)),
                Commands.print("[SIMTEST] shoot test"),
                Commands.deadline(
                    Commands.sequence(
                        Commands.waitSeconds(5.0),
                        Commands.runOnce(
                            () ->
                                System.out.printf(
                                    "[SIMTEST] shoot: ready=%b drumRpm=%.0f distValid=%b"
                                        + " dist=%.2f m shots=%d%n",
                                    shooter.isReadyToShoot(),
                                    shooter.getDrumVelocityRpm(),
                                    shooter.hasValidDistance(),
                                    vision.getHubDistanceMeters(),
                                    simulationManager.getShotsFired())),
                        Commands.waitSeconds(2.0)),
                    // Aim + shoot so the new aim interlock is satisfied (sticks read zero, so
                    // the robot rotates in place to face the hub)
                    aimAndShootCommand()),
                Commands.runOnce(
                    () ->
                        System.out.printf(
                            "[SIMTEST] SEQUENCE COMPLETE: shots=%d homed=%b retractedPos=%.4f m%n",
                            simulationManager.getShotsFired(),
                            intake.isHomed(),
                            intake.getLinearPositionMeters())))
            .withName("SimTestSequence");
    new Trigger(DriverStation::isTeleopEnabled).onTrue(testSequence);
  }

  /** Publishes 3D component poses; called from Robot.robotPeriodic after the scheduler runs. */
  public void updateVisualization() {
    robotVisualizer.update();
  }

  /**
   * Steps the maple-sim field simulation; called from Robot.simulationPeriodic (no-op in REPLAY).
   */
  public void updateSimulation() {
    if (simulationManager != null) {
      simulationManager.update();
    }
  }

  /** Restocks the simulated field's game pieces at the start of autonomous (no-op outside SIM). */
  public void resetSimulationField() {
    if (simulationManager != null) {
      simulationManager.resetFieldForAuto();
    }
  }
}
