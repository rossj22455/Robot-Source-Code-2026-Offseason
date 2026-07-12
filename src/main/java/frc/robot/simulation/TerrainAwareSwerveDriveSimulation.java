// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Arrays;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.mathutils.GeometryConvertor;
import org.ironmaple.utils.mathutils.MapleCommonMath;

/**
 * Swerve drive simulation with real 3D bump dynamics layered onto maple-sim's planar physics.
 *
 * <p>maple-sim (dyn4j) is strictly 2D, so instead of faking tilt with a kinematic lookup, this
 * models the chassis as a rigid body with three out-of-plane degrees of freedom — heave (z), pitch,
 * and roll — supported by a stiff spring-damper contact at each wheel, sampled against the {@link
 * FieldTerrain} height field. The vertical dynamics are coupled BOTH ways into the planar sim:
 *
 * <ul>
 *   <li><b>Slope reaction forces</b>: each wheel contact transmits a horizontal force of {@code
 *       -N_i * gradient} into the dyn4j body at that wheel's position, so climbing a ramp really
 *       slows the robot (and straddling a ramp edge really yaws it) instead of the chassis gliding
 *       over the terrain at full speed.
 *   <li><b>Per-wheel traction</b>: the live contact normal force {@code N_i} (not a constant mg/4)
 *       feeds each module's grip limit, so weight transfer under accel/braking and wheel unloading
 *       on the ramps change how much force each module can put down; a lifted wheel (N=0) propels
 *       nothing.
 * </ul>
 *
 * <p>Weight transfer emerges from the measured planar acceleration: traction forces act at floor
 * level, a CG height below them, so accelerating squats the chassis (nose up) and braking dives it
 * — with the resulting spring-damper transient visible on the simulated gyro. Sign conventions:
 * pitch is nose-up positive, roll is left-side-up positive.
 *
 * <p>The chassis friction / propulsion sub-tick logic is adapted from maple-sim 0.4.0-beta's {@code
 * SwerveDriveSimulation} (BSD), modified to use the per-wheel normal forces. If the vendordep is
 * ever bumped, re-diff against the library's {@code simulationSubTick}.
 *
 * <p>Honest limits: this is terrain dynamics, not full 3D collision — bumpers cannot hit ramp side
 * walls, game pieces stay in the 2D plane, and the model cannot roll the robot over.
 */
public class TerrainAwareSwerveDriveSimulation extends SwerveDriveSimulation {
  // --- Contact model (per wheel) ---
  // Tread-on-carpet + frame compliance modeled as a stiff spring: ~4 mm static deflection under
  // the 74 kg robot, ~8 Hz heave natural frequency, damping ratio ~0.5. Stable at maple-sim's
  // default 4 ms sub-tick (omega * dt ~= 0.2).
  private static final double CONTACT_STIFFNESS_N_PER_M = 45_000.0;
  private static final double CONTACT_DAMPING_N_S_PER_M = 950.0;

  // PLACEHOLDER estimates from CAD-ish numbers: CG height above the floor, and pitch/roll moments
  // of inertia (yaw MOI is 6.883 from PathPlanner config; pitch/roll are lower for a flat robot)
  private static final double CG_HEIGHT_METERS = 0.20;
  private static final double PITCH_MOI_KG_M2 = 4.5;
  private static final double ROLL_MOI_KG_M2 = 4.0;

  private static final double GRAVITY_MPS2 = 9.8; // matches maple-sim's internal value
  // Teleports (auto init, field reset) make the finite-differenced acceleration spike for one
  // sub-tick; clamp to a bit above the drivetrain's real capability so weight transfer can't blow
  // up the attitude state
  private static final double MAX_PLANAR_ACCEL_MPS2 = 20.0;
  // Attitude guards — far beyond anything the 5.3 deg ramps produce; only reachable through a
  // numerical glitch, in which case we saturate instead of diverging
  private static final double MAX_TILT_RAD = 0.6;
  private static final double MAX_HEAVE_METERS = 0.5;

  private final double massKg;
  private final double restCompressionMeters;

  // Out-of-plane rigid-body state (deviations from flat-ground rest)
  private double heaveMeters = 0.0;
  private double heaveVelMps = 0.0;
  private double pitchRad = 0.0; // nose-up positive
  private double pitchRateRadPerSec = 0.0;
  private double rollRad = 0.0; // left-side-up positive
  private double rollRateRadPerSec = 0.0;

  private final double[] normalForcesNewtons;
  private boolean onRamp = false;

  private Vector2 previousLinearVelocity = new Vector2();
  private Translation2d previousModuleSpeedsFieldRelative = new Translation2d();

  public TerrainAwareSwerveDriveSimulation(
      DriveTrainSimulationConfig config, Pose2d initialPoseOnField) {
    super(config, initialPoseOnField);
    this.massKg = config.robotMass.in(Kilograms);
    this.restCompressionMeters =
        massKg * GRAVITY_MPS2 / (moduleTranslations.length * CONTACT_STIFFNESS_N_PER_M);
    this.normalForcesNewtons = new double[moduleTranslations.length];
    // Start at static equilibrium so there is no settling transient on boot
    Arrays.fill(normalForcesNewtons, massKg * GRAVITY_MPS2 / moduleTranslations.length);
  }

  @Override
  public void simulationSubTick() {
    updateTerrainDynamics();

    simulateChassisFrictionForce();
    simulateChassisFrictionTorque();
    simulateModulePropellingForces();

    gyroSimulation.updateSimulationSubTick(super.getAngularVelocity());
  }

  /**
   * Integrates the heave/pitch/roll rigid-body dynamics one sub-tick: samples the terrain under
   * each wheel, computes the spring-damper contact normal forces, applies the down-slope reaction
   * forces to the planar body, and steps the attitude state (semi-implicit Euler).
   */
  private void updateTerrainDynamics() {
    final double dt = SimulatedArena.getSimulationDt().in(Seconds);
    final Rotation2d heading = getSimulatedDriveTrainPose().getRotation();

    // Planar acceleration (finite difference) for weight transfer, rotated into the robot frame
    final Vector2 velocity = super.getLinearVelocity().copy();
    final Vector2 accelWorld = velocity.difference(previousLinearVelocity).multiply(1.0 / dt);
    previousLinearVelocity = velocity;
    final double accelForward =
        MathUtil.clamp(
            accelWorld.x * heading.getCos() + accelWorld.y * heading.getSin(),
            -MAX_PLANAR_ACCEL_MPS2,
            MAX_PLANAR_ACCEL_MPS2);
    final double accelLeft =
        MathUtil.clamp(
            -accelWorld.x * heading.getSin() + accelWorld.y * heading.getCos(),
            -MAX_PLANAR_ACCEL_MPS2,
            MAX_PLANAR_ACCEL_MPS2);

    // Wheel contacts: normal forces from compression against the terrain height field
    double totalNormalForce = 0.0;
    double pitchMoment = 0.0;
    double rollMoment = 0.0;
    boolean anySlope = false;
    for (int i = 0; i < moduleTranslations.length; i++) {
      final Translation2d wheelRobot = moduleTranslations[i];
      final Vector2 wheelWorld = getWorldPoint(GeometryConvertor.toDyn4jVector2(wheelRobot));
      final double terrainHeight = FieldTerrain.heightAt(wheelWorld.x, wheelWorld.y);
      final Translation2d gradient = FieldTerrain.gradientAt(wheelWorld.x, wheelWorld.y);

      // Rate the terrain rises under the moving contact point
      final Vector2 wheelVelocity = super.getLinearVelocity(wheelWorld);
      final double terrainRate =
          gradient.getX() * wheelVelocity.x + gradient.getY() * wheelVelocity.y;

      // Wheel hub height (small-angle) and its rate, relative to flat-ground rest
      final double hubHeight =
          heaveMeters + wheelRobot.getX() * pitchRad + wheelRobot.getY() * rollRad;
      final double hubRate =
          heaveVelMps
              + wheelRobot.getX() * pitchRateRadPerSec
              + wheelRobot.getY() * rollRateRadPerSec;

      final double compression = restCompressionMeters + terrainHeight - hubHeight;
      final double compressionRate = terrainRate - hubRate;
      final double normalForce =
          Math.max(
              0.0,
              CONTACT_STIFFNESS_N_PER_M * compression
                  + CONTACT_DAMPING_N_S_PER_M * compressionRate);
      normalForcesNewtons[i] = normalForce;

      totalNormalForce += normalForce;
      pitchMoment += normalForce * wheelRobot.getX();
      rollMoment += normalForce * wheelRobot.getY();

      // The contact transmits the down-slope component of its normal force into the plane;
      // applied at the wheel position so straddling a ramp edge also yaws the chassis
      if (normalForce > 0.0 && (gradient.getX() != 0.0 || gradient.getY() != 0.0)) {
        anySlope = true;
        super.applyForce(
            new Vector2(-normalForce * gradient.getX(), -normalForce * gradient.getY()),
            wheelWorld);
      }
    }
    onRamp = anySlope;

    // Rigid-body attitude dynamics. Weight transfer: traction forces act at floor level, a CG
    // height below the chassis, so forward accel pitches the nose up (squat) and leftward accel
    // rolls the left side up (lean out of the turn).
    final double heaveAccel = totalNormalForce / massKg - GRAVITY_MPS2;
    final double pitchAccel =
        (pitchMoment + massKg * accelForward * CG_HEIGHT_METERS) / PITCH_MOI_KG_M2;
    final double rollAccel = (rollMoment + massKg * accelLeft * CG_HEIGHT_METERS) / ROLL_MOI_KG_M2;

    heaveVelMps += heaveAccel * dt;
    heaveMeters =
        MathUtil.clamp(heaveMeters + heaveVelMps * dt, -MAX_HEAVE_METERS, MAX_HEAVE_METERS);
    pitchRateRadPerSec += pitchAccel * dt;
    pitchRad = MathUtil.clamp(pitchRad + pitchRateRadPerSec * dt, -MAX_TILT_RAD, MAX_TILT_RAD);
    rollRateRadPerSec += rollAccel * dt;
    rollRad = MathUtil.clamp(rollRad + rollRateRadPerSec * dt, -MAX_TILT_RAD, MAX_TILT_RAD);
  }

  // -----------------------------------------------------------------------------------------
  // Chassis friction / propulsion, adapted from maple-sim 0.4.0-beta SwerveDriveSimulation with
  // the fixed mg/4 module load replaced by the live per-wheel contact normal forces.
  // -----------------------------------------------------------------------------------------

  private void simulateChassisFrictionForce() {
    final ChassisSpeeds moduleSpeeds = getModuleSpeeds();

    /* The friction force that tries to bring the chassis from floor speeds to module speeds */
    final ChassisSpeeds differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative =
        moduleSpeeds.minus(getDriveTrainSimulatedChassisSpeedsRobotRelative());
    final Translation2d floorAndModuleSpeedsDiffFieldRelative =
        new Translation2d(
                differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative.vxMetersPerSecond,
                differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative.vyMetersPerSecond)
            .rotateBy(getSimulatedDriveTrainPose().getRotation());
    final double FRICTION_FORCE_GAIN = 3.0;
    final double totalGrippingForce = getTotalGrippingForceNewtons();
    final Vector2 speedsDifferenceFrictionForce =
        Vector2.create(
            Math.min(
                FRICTION_FORCE_GAIN
                    * totalGrippingForce
                    * floorAndModuleSpeedsDiffFieldRelative.getNorm(),
                totalGrippingForce),
            MapleCommonMath.getAngle(floorAndModuleSpeedsDiffFieldRelative).getRadians());

    /* the centripetal friction force during turning */
    final ChassisSpeeds moduleSpeedsFieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            moduleSpeeds, getSimulatedDriveTrainPose().getRotation());
    final Rotation2d dTheta =
        MapleCommonMath.getAngle(
                GeometryConvertor.getChassisSpeedsTranslationalComponent(moduleSpeedsFieldRelative))
            .minus(MapleCommonMath.getAngle(previousModuleSpeedsFieldRelative));

    final double orbitalAngularVelocity =
        dTheta.getRadians() / SimulatedArena.getSimulationDt().in(Seconds);
    final Rotation2d centripetalForceDirection =
        MapleCommonMath.getAngle(previousModuleSpeedsFieldRelative)
            .plus(Rotation2d.fromDegrees(90));
    final Vector2 centripetalFrictionForce =
        Vector2.create(
            previousModuleSpeedsFieldRelative.getNorm() * orbitalAngularVelocity * massKg,
            centripetalForceDirection.getRadians());
    previousModuleSpeedsFieldRelative =
        GeometryConvertor.getChassisSpeedsTranslationalComponent(moduleSpeedsFieldRelative);

    /* apply force to physics engine */
    final Vector2 totalFrictionForceUnlimited =
        centripetalFrictionForce.copy().add(speedsDifferenceFrictionForce);
    final Vector2 totalFrictionForce =
        Vector2.create(
            Math.min(totalGrippingForce, totalFrictionForceUnlimited.getMagnitude()),
            totalFrictionForceUnlimited.getDirection());
    super.applyForce(totalFrictionForce);
  }

  private void simulateChassisFrictionTorque() {
    final SwerveModuleSimulation[] modules = getModules();
    double grippingTorqueMagnitude = 0.0;
    for (int i = 0; i < modules.length; i++) {
      grippingTorqueMagnitude +=
          modules[i].config.getGrippingForceNewtons(normalForcesNewtons[i])
              * moduleTranslations[i].getNorm();
    }

    final double desiredRotationalMotionPercent =
        Math.abs(
            getDesiredSpeed().omegaRadiansPerSecond / maxAngularVelocity().in(RadiansPerSecond));
    final double actualRotationalMotionPercent =
        Math.abs(getAngularVelocity() / maxAngularVelocity().in(RadiansPerSecond));
    final double differenceBetweenFloorSpeedAndModuleSpeed =
        getModuleSpeeds().omegaRadiansPerSecond - getAngularVelocity();
    final double FRICTION_TORQUE_GAIN = 1;

    if (actualRotationalMotionPercent < 0.01 && desiredRotationalMotionPercent < 0.02)
      super.setAngularVelocity(0);
    else
      super.applyTorque(
          Math.copySign(
              Math.min(
                  FRICTION_TORQUE_GAIN
                      * grippingTorqueMagnitude
                      * Math.abs(differenceBetweenFloorSpeedAndModuleSpeed),
                  grippingTorqueMagnitude),
              differenceBetweenFloorSpeedAndModuleSpeed));
  }

  private void simulateModulePropellingForces() {
    final SwerveModuleSimulation[] modules = getModules();
    for (int i = 0; i < modules.length; i++) {
      final Vector2 moduleWorldPosition =
          getWorldPoint(GeometryConvertor.toDyn4jVector2(moduleTranslations[i]));
      final Vector2 moduleForce =
          modules[i].updateSimulationSubTickGetModuleForce(
              super.getLinearVelocity(moduleWorldPosition),
              getSimulatedDriveTrainPose().getRotation(),
              normalForcesNewtons[i]);
      super.applyForce(moduleForce, moduleWorldPosition);
    }
  }

  private double getTotalGrippingForceNewtons() {
    final SwerveModuleSimulation[] modules = getModules();
    double total = 0.0;
    for (int i = 0; i < modules.length; i++) {
      total += modules[i].config.getGrippingForceNewtons(normalForcesNewtons[i]);
    }
    return total;
  }

  private ChassisSpeeds getModuleSpeeds() {
    return kinematics.toChassisSpeeds(
        Arrays.stream(getModules())
            .map(SwerveModuleSimulation::getCurrentState)
            .toArray(SwerveModuleState[]::new));
  }

  /** Free-spin chassis speeds (library's private getDesiredSpeed, rebuilt from public API). */
  private ChassisSpeeds getDesiredSpeed() {
    return kinematics.toChassisSpeeds(
        Arrays.stream(getModules())
            .map(TerrainAwareSwerveDriveSimulation::getFreeSpinState)
            .toArray(SwerveModuleState[]::new));
  }

  private static SwerveModuleState getFreeSpinState(SwerveModuleSimulation module) {
    return new SwerveModuleState(
        module
                .config
                .driveMotorConfigs
                .calculateMechanismVelocity(
                    module.config.driveMotorConfigs.calculateCurrent(
                        module.config.driveMotorConfigs.friction),
                    module.getDriveMotorAppliedVoltage())
                .in(RadiansPerSecond)
            * module.config.WHEEL_RADIUS.in(Meters),
        module.getSteerAbsoluteFacing());
  }

  // -----------------------------------------------------------------------------------------
  // Terrain state accessors (ground truth; the sim gyro reads pitch/roll through these)
  // -----------------------------------------------------------------------------------------

  /** Chassis pitch, nose-up positive. */
  public Rotation2d getPitch() {
    return Rotation2d.fromRadians(pitchRad);
  }

  /** Chassis roll, left-side-up positive. */
  public Rotation2d getRoll() {
    return Rotation2d.fromRadians(rollRad);
  }

  /** Chassis vertical displacement from flat-ground ride height (for the 3D robot pose). */
  public double getHeaveMeters() {
    return heaveMeters;
  }

  /** Live contact normal force per wheel (module order), newtons; 0 means the wheel is airborne. */
  public double[] getWheelNormalForcesNewtons() {
    return normalForcesNewtons.clone();
  }

  /** Whether any wheel is currently on a ramp slope. */
  public boolean isOnRamp() {
    return onRamp;
  }
}
