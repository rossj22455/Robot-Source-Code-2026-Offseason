// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.Constants.Shooter.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Dual-mechanism shooter with a fixed hood: a drum flywheel (2x Falcon 500, onboard Phoenix 6
 * velocity PID, hardware follower) and a kicker (NEO) that pushes balls from the indexer entrance
 * up into the drum.
 *
 * <p>The target RPM is computed exclusively from the vision system's hub distance through an {@link
 * InterpolatingDoubleTreeMap}. {@link #isReadyToShoot()} is the system-wide anti-jamming interlock:
 * the kicker (here) and the indexer are forbidden from feeding into the drum unless it returns
 * true.
 */
public class Shooter extends SubsystemBase {
  private final ShooterDrumIO drumIO;
  private final ShooterKickerIO kickerIO;
  private final ShooterDrumIOInputsAutoLogged drumInputs = new ShooterDrumIOInputsAutoLogged();
  private final ShooterKickerIOInputsAutoLogged kickerInputs =
      new ShooterKickerIOInputsAutoLogged();

  // Vision distance channel (bound to Vision::getHubDistanceMeters / hasHubPoseConfidence)
  private final DoubleSupplier hubDistanceSupplier;
  private final BooleanSupplier distanceValidSupplier;

  // Aim channel: the hood is fixed, so being aimed means the robot heading matches the target
  // bearing (hub normally; the alliance corner while funneling from the neutral zone)
  private final Supplier<Rotation2d> robotHeadingSupplier;
  private final Supplier<Rotation2d> targetHeadingSupplier;

  // True while the robot is inside the neutral zone, where hub shots are illegal and fuel is
  // funneled toward the alliance corner at a fixed RPM instead
  private final BooleanSupplier funnelModeSupplier;

  // Distance (meters) -> drum RPM, built once from constants to avoid periodic allocation
  private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
  private final Debouncer readyDebouncer = new Debouncer(DRUM_READY_DEBOUNCE_SECS);

  private boolean spinUpRequested = false;
  // Manual test-shot RPM override (NaN = off). When set, the drum targets this RPM instead of the
  // vision/fallback value and the aim check is skipped (the robot does not rotate for these shots)
  private double manualRpm = Double.NaN;
  private boolean unjamRequested = false;
  private double targetRpm = 0.0;
  private boolean readyToShoot = false;

  private final Alert drumDisconnectedAlert =
      new Alert("Shooter drum master is disconnected.", AlertType.kError);
  private final Alert drumFollowerDisconnectedAlert =
      new Alert("Shooter drum follower is disconnected.", AlertType.kError);
  private final Alert kickerDisconnectedAlert =
      new Alert("Shooter kicker is disconnected.", AlertType.kWarning);
  private final Alert drumTempAlert =
      new Alert("Shooter drum motor is overheating.", AlertType.kWarning);
  private final Alert rpmMapAlert =
      new Alert(
          "Shooter DISTANCE_TO_RPM_MAP has fewer than 2 distinct distance keys; the"
              + " interpolation table cannot interpolate and returns a constant RPM.",
          AlertType.kError);
  private final Alert visionFallbackAlert =
      new Alert(
          "Shooter spinning without recent vision — using the fixed fallback RPM.",
          AlertType.kWarning);

  public Shooter(
      ShooterDrumIO drumIO,
      ShooterKickerIO kickerIO,
      DoubleSupplier hubDistanceSupplier,
      BooleanSupplier distanceValidSupplier,
      Supplier<Rotation2d> robotHeadingSupplier,
      Supplier<Rotation2d> targetHeadingSupplier,
      BooleanSupplier funnelModeSupplier) {
    this.drumIO = drumIO;
    this.kickerIO = kickerIO;
    this.hubDistanceSupplier = hubDistanceSupplier;
    this.distanceValidSupplier = distanceValidSupplier;
    this.robotHeadingSupplier = robotHeadingSupplier;
    this.targetHeadingSupplier = targetHeadingSupplier;
    this.funnelModeSupplier = funnelModeSupplier;

    for (double[] point : DISTANCE_TO_RPM_MAP) {
      rpmMap.put(point[0], point[1]);
    }

    // Duplicate distance keys silently overwrite each other in the InterpolatingDoubleTreeMap,
    // collapsing the table to a constant. Fail loudly: latched alert on the real robot, hard
    // stop in simulation so it can never slip through desktop testing.
    long distinctDistances =
        Arrays.stream(DISTANCE_TO_RPM_MAP).mapToDouble(point -> point[0]).distinct().count();
    if (distinctDistances < 2) {
      rpmMapAlert.set(true);
      if (Constants.currentMode == Constants.Mode.SIM) {
        throw new IllegalStateException(
            "Shooter DISTANCE_TO_RPM_MAP must contain at least 2 distinct distance keys");
      }
    }
  }

  @Override
  public void periodic() {
    drumIO.updateInputs(drumInputs);
    kickerIO.updateInputs(kickerInputs);
    Logger.processInputs("Shooter/Drum", drumInputs);
    Logger.processInputs("Shooter/Kicker", kickerInputs);

    // Health monitoring
    drumDisconnectedAlert.set(!drumInputs.connected);
    drumFollowerDisconnectedAlert.set(!drumInputs.followerConnected);
    kickerDisconnectedAlert.set(!kickerInputs.connected);
    drumTempAlert.set(
        Math.max(drumInputs.tempCelsius, drumInputs.followerTempCelsius)
            > MOTOR_TEMP_WARNING_CELSIUS);

    // State update: compute the vision-mapped target and command the onboard velocity loop
    if (DriverStation.isDisabled()) {
      spinUpRequested = false;
      unjamRequested = false;
      manualRpm = Double.NaN;
    }
    boolean manual = !Double.isNaN(manualRpm);
    // Target RPM source, in priority order. Vision can never be relied on, so losing it must
    // never disable the shooter — it only degrades to a fixed setpoint:
    // 1. FUNNELING (in the neutral zone, hub shots illegal): fixed corner-lob RPM,
    //    vision-independent.
    // 2. HUB_VISION: distance-mapped RPM from the vision-corrected pose.
    // 3. HUB_FALLBACK (vision stale): fixed fallback RPM; the driver ranges by eye.
    boolean funneling = funnelModeSupplier.getAsBoolean();
    boolean visionValid = distanceValidSupplier.getAsBoolean();
    String targetingMode =
        manual
            ? "MANUAL_TEST"
            : (funneling ? "FUNNELING" : (visionValid ? "HUB_VISION" : "HUB_FALLBACK"));
    if (unjamRequested) {
      // Chute unjam (highest priority): spin the drum FORWARD a little faster than idle to fling
      // a ball stuck in the chute clear, and reverse the kicker at a moderate speed to back the
      // jam out. Not a shot — targetRpm stays 0 so the ready-to-shoot interlock never engages.
      targetRpm = 0.0;
      drumIO.setVelocityRpm(UNJAM_DRUM_RPM);
      kickerIO.setVoltage(KICKER_REVERSE_VOLTS);
    } else if (spinUpRequested) {
      if (manual) {
        targetRpm = manualRpm;
      } else if (funneling) {
        targetRpm = FUNNEL_RPM;
      } else if (visionValid) {
        targetRpm = rpmMap.get(hubDistanceSupplier.getAsDouble());
      } else {
        targetRpm = VISION_FALLBACK_RPM;
      }
      drumIO.setVelocityRpm(targetRpm);
    } else {
      // Not shooting: keep the shot target at 0 (so the ready-to-shoot interlock stays false),
      // stop feeding, but hold the drum at a low idle spin while enabled so the next spin-up is
      // a small step rather than from a dead stop. Coast when disabled (motors can't move anyway).
      targetRpm = 0.0;
      kickerIO.setVoltage(0.0);
      // Idle spin disabled for tuning: coast the drum instead. Something must command the drum
      // here, otherwise the Talon keeps running its last shot velocity request indefinitely.
      drumIO.stop();
      // if (DriverStation.isEnabled()) {
      //   drumIO.setVelocityRpm(SHOOTER_IDLE_RPM);
      // } else {
      //   drumIO.stop();
      // }
    }
    visionFallbackAlert.set(spinUpRequested && !manual && !funneling && !visionValid);

    // Anti-jamming interlock: ready only when the drum has stabilized within a razor-thin
    // tolerance of the target RPM (debounced against momentary crossings) AND the robot is
    // aimed at the current target (fixed hood: heading IS aim; hub normally, alliance corner
    // while funneling). Vision freshness deliberately does NOT gate here — losing vision falls
    // back to a fixed RPM instead of disabling the shooter. Feeding is blocked at the interlock
    // root, covering the kicker AND the indexer, which both key off isReadyToShoot().
    double aimErrorDeg =
        Math.abs(robotHeadingSupplier.get().minus(targetHeadingSupplier.get()).getDegrees());
    boolean aimedAtTarget = aimErrorDeg <= AIM_TOLERANCE_DEG;
    boolean shotConditions =
        spinUpRequested
            && targetRpm > 0.0
            && (aimedAtTarget || manual); // Manual test shots fire wherever the robot faces
    // Entry: the drum must settle within the tight tolerance (debounced) before feeding starts
    boolean enteredReady =
        readyDebouncer.calculate(
            shotConditions
                && Math.abs(drumInputs.velocityRpm - targetRpm) <= DRUM_READY_TOLERANCE_RPM);
    // Hold: once feeding, keep going through the per-ball RPM dips unless the drum sags past the
    // drop allowance (hysteresis — prevents the feed stuttering on and off with every ball)
    boolean holdingReady = drumInputs.velocityRpm >= targetRpm - DRUM_READY_DROP_ALLOWANCE_RPM;
    readyToShoot = shotConditions && holdingReady && (readyToShoot || enteredReady);

    // Telemetry
    Logger.recordOutput("Shooter/Drum/TargetRpm", targetRpm);
    Logger.recordOutput("Shooter/Drum/VelocityErrorRpm", drumInputs.velocityRpm - targetRpm);
    Logger.recordOutput("Shooter/ReadyToShoot", readyToShoot);
    Logger.recordOutput("Shooter/SpinUpRequested", spinUpRequested);
    Logger.recordOutput("Shooter/UnjamActive", unjamRequested);
    Logger.recordOutput("Shooter/HubDistanceMeters", hubDistanceSupplier.getAsDouble());
    Logger.recordOutput("Shooter/DistanceValid", visionValid);
    Logger.recordOutput("Shooter/TargetingMode", targetingMode);
    Logger.recordOutput("Shooter/AimErrorDeg", aimErrorDeg);
    Logger.recordOutput("Shooter/AimedAtTarget", aimedAtTarget);
  }

  /** Starts spinning the drum to the vision-mapped RPM for the current hub distance. */
  public void startSpinUp() {
    spinUpRequested = true;
  }

  /**
   * TESTING: spins the drum to a fixed RPM (clamped to 0-5000) and lets the feed interlock ignore
   * aim, so shots fire in whatever direction the robot faces. Cleared by {@link #stopShooter()}.
   */
  public void startManualSpinUp(double rpm) {
    manualRpm = MathUtil.clamp(rpm, 0.0, 5000.0);
    spinUpRequested = true;
  }

  /** Stops the drum (coast) and the kicker. */
  public void stopShooter() {
    spinUpRequested = false;
    manualRpm = Double.NaN;
  }

  /**
   * Anti-jamming interlock. True only when the drum's real-time velocity has stabilized within
   * {@link Constants.Shooter#DRUM_READY_TOLERANCE_RPM} of the vision-mapped target RPM. The kicker
   * and the indexer must never feed into the drum while this is false.
   */
  public boolean isReadyToShoot() {
    return readyToShoot;
  }

  /**
   * Returns true when the vision pose is fresh enough to trust the mapped RPM; commands should gate
   * automatic firing on this in addition to {@link #isReadyToShoot()}.
   */
  public boolean hasValidDistance() {
    return distanceValidSupplier.getAsBoolean();
  }

  /**
   * Runs the kicker forward to feed a ball into the drum. Enforces the interlock internally: the
   * kicker will not move unless {@link #isReadyToShoot()} is true.
   */
  public void runKickerFeed() {
    kickerIO.setVoltage(isReadyToShoot() ? KICKER_FEED_VOLTS : 0.0);
  }

  /**
   * Enters chute-unjam mode: the drum spins forward a little faster than idle to fling a stuck ball
   * clear while the kicker reverses at a moderate speed to back the jam out. Handled in {@link
   * #periodic()} so it cleanly overrides idle/spin-up. Pair with the indexer's reverse for a full
   * clear.
   */
  public void startUnjam() {
    unjamRequested = true;
  }

  /** Exits chute-unjam mode; the drum returns to idle spin and the kicker stops. */
  public void stopUnjam() {
    unjamRequested = false;
  }

  /** Stops the kicker. */
  public void stopKicker() {
    kickerIO.setVoltage(0.0);
  }

  /** Current drum surface velocity in RPM (drum rotations, not motor rotations). */
  public double getDrumVelocityRpm() {
    return drumInputs.velocityRpm;
  }

  /** Voltage currently applied to the kicker motor (used by the simulation manager). */
  public double getKickerAppliedVolts() {
    return kickerInputs.appliedVolts;
  }

  /** Kicker output-shaft velocity in RPM (used by the 3D visualizer). */
  public double getKickerVelocityRpm() {
    return kickerInputs.velocityRpm;
  }

  /** Simulation hook: forwards the per-ball drum spindown impulse (no-op outside sim). */
  public void simNotifyBallFired() {
    drumIO.simNotifyBallFired();
  }
}
