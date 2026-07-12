// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * SIM-only orchestrator for the maple-sim field: binds the intake collision volume to the real
 * intake's extension and roller state, converts kicker feeds into launched Fuel projectiles at the
 * drum's surface speed, steps the arena physics, and publishes game-piece / ground-truth poses for
 * AdvantageScope. Constructed only in {@code Constants.Mode.SIM}; never on the real robot or in
 * replay.
 */
public class SimulationManager {
  // --- Sim-only tuning ---
  private static final double INTAKE_WIDTH_METERS = Units.inchesToMeters(23.66);
  private static final IntakeSimulation.IntakeSide INTAKE_SIDE = IntakeSimulation.IntakeSide.BACK;
  private static final double ROLLER_ACTIVE_VOLTS = 1.0;
  private static final double INTAKE_DEPLOYED_FRACTION = 0.8;

  private static final int HOPPER_CAPACITY = 49;
  private static final Translation3d BALL_PATH_START = new Translation3d(-0.45, 0.0, 0.20);
  private static final Translation3d BALL_PATH_END = new Translation3d(0.05, 0.0, 0.40);
  private static final double TRANSPORT_SPEED_MPS = 0.75;
  private static final double INDEXER_ACTIVE_VOLTS = 1.0;
  private static final double FUEL_DIAMETER_METERS = 0.150;

  // Shot model
  private static final double KICKER_FEED_THRESHOLD_VOLTS = 1.0;
  // Default commanded flow rate; live-tune via /Tuning/FieldSim/BallsPerSecond. With 3-wide
  // volleys, BPS 6 = one volley every 0.5 s. The drum-recovery interlock still applies: if the
  // drum can't recover between volleys, the achieved rate is lower (that's the robot's real
  // limit, not the feed's).
  private static final double DEFAULT_BALLS_PER_SECOND = 18;
  // Physical shot geometry lives in ShooterConstants (shared with the real robot's live
  // trajectory prediction in RobotVisualizer); aliased here for readability
  private static final double SHOT_EFFICIENCY = ShooterConstants.SURFACE_TO_BALL_SPEED_RATIO;
  private static final double SHOT_ANGLE_DEG = ShooterConstants.HOOD_ANGLE_DEG;
  private static final double SHOT_INITIAL_HEIGHT_METERS = ShooterConstants.BALL_EXIT_HEIGHT_METERS;
  private static final Translation2d SHOOTER_POSITION_ON_ROBOT = ShooterConstants.BALL_EXIT_OFFSET;

  // 3-wide shooter lane pitch: the drum is 21.004 in wide, so three lanes sit at 0 and
  // +/- 7.0 in (21.004 / 3)
  private static final double SHOOTER_LANE_SPREAD_METERS = Units.inchesToMeters(21.004 / 3.0);

  private static final boolean FULL_FIELD_FUEL = false;
  private static final double SPAWN_DISTANCE_METERS = 1.0;

  private final SwerveDriveSimulation driveSimulation;
  private final DoubleSupplier intakePositionMeters;
  private final DoubleSupplier rollerAppliedVolts;
  private final DoubleSupplier drumVelocityRpm;
  private final DoubleSupplier kickerAppliedVolts;
  private final DoubleSupplier indexerAppliedVolts;
  private final Runnable ballFiredCallback;
  private final BumpSimulation bumpSimulation;
  private final IntakeSimulation intakeSimulation;

  private final List<Double> ballProgress = new ArrayList<>();

  private final LoggedNetworkBoolean spawnFuelAtIntakeRequest =
      new LoggedNetworkBoolean("/Tuning/FieldSim/SpawnFuelAtIntake", false);
  private final LoggedNetworkBoolean restockFieldRequest =
      new LoggedNetworkBoolean("/Tuning/FieldSim/RestockField", false);
  private final LoggedNetworkBoolean outpostDumpRequest =
      new LoggedNetworkBoolean("/Tuning/FieldSim/OutpostDumpBlue", false);
  private final org.littletonrobotics.junction.networktables.LoggedNetworkNumber ballsPerSecond =
      new org.littletonrobotics.junction.networktables.LoggedNetworkNumber(
          "/Tuning/FieldSim/BallsPerSecond", DEFAULT_BALLS_PER_SECOND);

  private double lastShotTimestamp = -10.0;
  private int shotsFired = 0;

  public SimulationManager(
      SwerveDriveSimulation driveSimulation,
      DoubleSupplier intakePositionMeters,
      DoubleSupplier rollerAppliedVolts,
      DoubleSupplier drumVelocityRpm,
      DoubleSupplier kickerAppliedVolts,
      DoubleSupplier indexerAppliedVolts,
      Runnable ballFiredCallback,
      BumpSimulation bumpSimulation) {
    this.driveSimulation = driveSimulation;
    this.intakePositionMeters = intakePositionMeters;
    this.rollerAppliedVolts = rollerAppliedVolts;
    this.drumVelocityRpm = drumVelocityRpm;
    this.kickerAppliedVolts = kickerAppliedVolts;
    this.indexerAppliedVolts = indexerAppliedVolts;
    this.ballFiredCallback = ballFiredCallback;
    this.bumpSimulation = bumpSimulation;

    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveSimulation,
            Meters.of(INTAKE_WIDTH_METERS),
            Meters.of(
                IntakeConstants.LINEAR_MAX_POSITION_METERS
                    * Math.cos(IntakeConstants.RACK_ANGLE_RAD)),
            INTAKE_SIDE,
            1);

    if (SimulatedArena.getInstance() instanceof Arena2026Rebuilt arena) {
      arena.setEfficiencyMode(!FULL_FIELD_FUEL);
    }
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void update() {
    handleFieldTestControls();

    // Synthetic ramp terrain -> gyro pitch/roll (tilt logic testable in sim)
    bumpSimulation.update(driveSimulation.getSimulatedDriveTrainPose());

    boolean intakeActive =
        rollerAppliedVolts.getAsDouble() > ROLLER_ACTIVE_VOLTS
            && intakePositionMeters.getAsDouble()
                > INTAKE_DEPLOYED_FRACTION * IntakeConstants.LINEAR_EXTENDED_POSITION_METERS
            && ballProgress.size() < HOPPER_CAPACITY;

    if (intakeActive) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }

    while (ballProgress.size() < HOPPER_CAPACITY && intakeSimulation.obtainGamePieceFromIntake()) {
      ballProgress.add(0.0);
    }

    // Commanded flow rate: the volley interval comes straight from BPS, and the belt speed
    // scales with it (1.5x margin) so restaging never starves the commanded rate
    double bps = Math.max(0.1, ballsPerSecond.get());
    double volleyIntervalSecs = 3.0 / bps;
    double transportSpeedMps = Math.max(TRANSPORT_SPEED_MPS, FUEL_DIAMETER_METERS * bps * 1.5);

    // The staging area at the end of the path holds a full 3-wide volley side by side; balls
    // further back queue single-file behind it
    double pathLengthMeters = BALL_PATH_END.getDistance(BALL_PATH_START);
    double spacingFraction = FUEL_DIAMETER_METERS / pathLengthMeters;
    boolean indexerFeeding = indexerAppliedVolts.getAsDouble() > INDEXER_ACTIVE_VOLTS;
    for (int i = 0; i < ballProgress.size(); i++) {
      double maxProgress = i < 3 ? 1.0 : 1.0 - (i - 2) * spacingFraction;
      double progress = ballProgress.get(i);
      if (indexerFeeding) {
        progress += transportSpeedMps * 0.02 / pathLengthMeters;
      }
      ballProgress.set(i, Math.min(progress, maxProgress));
    }

    // Wait for the staging rack to fill (3 wide, or everything left in the hopper) so every
    // volley fires full — otherwise the gate reopens the moment the first ball restages
    int stagedCount = 0;
    while (stagedCount < 3
        && stagedCount < ballProgress.size()
        && ballProgress.get(stagedCount) >= 0.999) {
      stagedCount++;
    }
    boolean volleyReady = stagedCount > 0 && stagedCount == Math.min(3, ballProgress.size());

    boolean kickerFeeding = kickerAppliedVolts.getAsDouble() > KICKER_FEED_THRESHOLD_VOLTS;
    double now = Timer.getFPGATimestamp();
    if (kickerFeeding && now - lastShotTimestamp >= volleyIntervalSecs && volleyReady) {
      launchFuel(); // Fires every staged ball (up to 3) and applies spindown per ball
      lastShotTimestamp = now;
    }

    // (The live predicted-trajectory parabola is published by RobotVisualizer, which runs in
    // ALL modes so the real robot gets it too)

    SimulatedArena.getInstance().simulationPeriodic();

    Logger.recordOutput(
        "FieldSimulation/GamePieces",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    Logger.recordOutput(
        "FieldSimulation/SimulatedRobotPose", driveSimulation.getSimulatedDriveTrainPose());
    // Ground truth as a full 3D pose riding the ramp terrain: bind the AdvantageScope 3D robot
    // to this key to see it climb and tilt over the bumps. (Rotation3d pitch is nose-DOWN
    // positive, hence the negation; flip signs if the tilt renders backward.)
    Pose2d groundTruth = driveSimulation.getSimulatedDriveTrainPose();
    Logger.recordOutput(
        "FieldSimulation/SimulatedRobotPose3d",
        new Pose3d(
            groundTruth.getX(),
            groundTruth.getY(),
            bumpSimulation.getHeightMeters(),
            new Rotation3d(
                bumpSimulation.getRoll().getRadians(),
                -bumpSimulation.getPitch().getRadians(),
                groundTruth.getRotation().getRadians())));
    Logger.recordOutput("FieldSimulation/BallsInRobot", getBallPosesInField());
    Logger.recordOutput("FieldSimulation/PiecesInRobot", ballProgress.size());
    Logger.recordOutput("FieldSimulation/ShotsFired", shotsFired);
    Logger.recordOutput("FieldSimulation/BlueScore", SimulatedArena.getInstance().getScore(true));
    Logger.recordOutput("FieldSimulation/RedScore", SimulatedArena.getInstance().getScore(false));
  }

  private void handleFieldTestControls() {
    if (spawnFuelAtIntakeRequest.get()) {
      spawnFuelAtIntakeRequest.set(false);
      Translation2d spawnSpot =
          driveSimulation
              .getSimulatedDriveTrainPose()
              .transformBy(new Transform2d(-SPAWN_DISTANCE_METERS, 0.0, Rotation2d.kZero))
              .getTranslation();
      SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(spawnSpot));
    }
    if (restockFieldRequest.get()) {
      restockFieldRequest.set(false);
      SimulatedArena.getInstance().resetFieldForAuto();
    }
    if (outpostDumpRequest.get()) {
      outpostDumpRequest.set(false);
      if (SimulatedArena.getInstance() instanceof Arena2026Rebuilt arena) {
        arena.outpostDump(true);
      }
    }
  }

  private Pose3d[] getBallPosesInField() {
    Pose3d robotPose = new Pose3d(driveSimulation.getSimulatedDriveTrainPose());
    int[] laneAssignments = {0, -1, 1};
    int stagedIndex = 0;
    Pose3d[] poses = new Pose3d[ballProgress.size()];
    for (int i = 0; i < poses.length; i++) {
      Translation3d ballInRobot = BALL_PATH_START.interpolate(BALL_PATH_END, ballProgress.get(i));
      // Staged balls spread across the 3-wide shooter lanes instead of stacking on one point
      if (ballProgress.get(i) >= 0.999 && stagedIndex < 3) {
        ballInRobot =
            ballInRobot.plus(
                new Translation3d(
                    0.0, laneAssignments[stagedIndex] * SHOOTER_LANE_SPREAD_METERS, 0.0));
        stagedIndex++;
      }
      poses[i] = robotPose.transformBy(new Transform3d(ballInRobot, Rotation3d.kZero));
    }
    return poses;
  }

  private void launchFuel() {
    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    double drumSurfaceSpeedMps =
        Math.abs(drumVelocityRpm.getAsDouble())
            / 60.0
            * 2.0
            * Math.PI
            * ShooterConstants.DRUM_RADIUS_METERS;
    double launchSpeedMps = Math.max(1.0, drumSurfaceSpeedMps * SHOT_EFFICIENCY);

    // A volley fires only the balls that have actually reached the staging area (up to 3);
    // balls still riding the hopper path stay put
    int ballsToShoot = 0;
    while (ballsToShoot < 3
        && ballsToShoot < ballProgress.size()
        && ballProgress.get(ballsToShoot) >= 0.999) {
      ballsToShoot++;
    }

    // Offsets for Left Lane (-1), Center Lane (0), Right Lane (1)
    int[] laneAssignments = {0, -1, 1};

    for (int i = 0; i < ballsToShoot; i++) {
      ballProgress.remove(0); // Pull the front ball
      shotsFired++;
      ballFiredCallback.run(); // Each ball robs the drum of its own 4.93% spindown

      // Apply lateral offset for the specific lane
      double lateralOffsetY = laneAssignments[i] * SHOOTER_LANE_SPREAD_METERS;
      Translation2d offsetShooterPosition =
          SHOOTER_POSITION_ON_ROBOT.plus(new Translation2d(0, lateralOffsetY));

      var fuel =
          new RebuiltFuelOnFly(
              robotPose.getTranslation(),
              offsetShooterPosition,
              driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
              robotPose.getRotation(),
              Meters.of(SHOT_INITIAL_HEIGHT_METERS),
              MetersPerSecond.of(launchSpeedMps),
              Degrees.of(SHOT_ANGLE_DEG));

      // Publish flight path of the center shot (lane 0) so we don't clutter A-Scope with 3
      // overlapping arcs
      if (i == 0) {
        fuel.withProjectileTrajectoryDisplayCallBack(
            (List<Pose3d> trajectory) ->
                Logger.recordOutput(
                    "FieldSimulation/ShotTrajectory", trajectory.toArray(new Pose3d[0])));
      }

      SimulatedArena.getInstance().addGamePieceProjectile(fuel);
    }

    Logger.recordOutput("FieldSimulation/LastShotSpeedMps", launchSpeedMps);
    Logger.recordOutput("FieldSimulation/LastShotAngleDeg", SHOT_ANGLE_DEG);
  }

  public void resetFieldForAuto() {
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void preloadFuel(int count) {
    double pathLengthMeters = BALL_PATH_END.getDistance(BALL_PATH_START);
    double spacingFraction = FUEL_DIAMETER_METERS / pathLengthMeters;
    for (int i = 0; i < count && ballProgress.size() < HOPPER_CAPACITY; i++) {
      ballProgress.add(1.0 - ballProgress.size() * spacingFraction);
    }
  }

  public int getShotsFired() {
    return shotsFired;
  }
}
