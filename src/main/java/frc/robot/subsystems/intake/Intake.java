// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Intake with a linear rack articulation (NEO, RIO-side profiled PID) and a dual-roller bar (NEO
 * master + inverted hardware follower).
 *
 * <p>Linear articulation lifecycle: on the first enable after boot the mechanism homes by driving
 * slowly into the retracted hardstop until a stall (current rise + velocity stagnation) is
 * detected, then zeros the encoder. Position control is unavailable until homing succeeds. While
 * holding position against the hardstop, the smart current limit is dropped heavily to protect the
 * NEO; it is restored while moving.
 */
public class Intake extends SubsystemBase {
  /** Linear articulation state machine. */
  public enum LinearState {
    /** Not yet homed; waiting for an enable edge to begin homing. */
    UNHOMED,
    /** Driving into the hardstop watching for a stall. */
    HOMING,
    /** Homed; RIO-side profiled PID position control is active. */
    RUNNING
  }

  private final IntakeLinearIO linearIO;
  private final IntakeRollerIO rollerIO;
  private final IntakeLinearIOInputsAutoLogged linearInputs = new IntakeLinearIOInputsAutoLogged();
  private final IntakeRollerIOInputsAutoLogged rollerInputs = new IntakeRollerIOInputsAutoLogged();

  private final ProfiledPIDController linearController =
      new ProfiledPIDController(
          LINEAR_P,
          LINEAR_I,
          LINEAR_D,
          new TrapezoidProfile.Constraints(
              LINEAR_MAX_VELOCITY_METERS_PER_SEC, LINEAR_MAX_ACCELERATION_METERS_PER_SEC_SQ));
  private final Debouncer stallDebouncer = new Debouncer(HOMING_STALL_DEBOUNCE_SECS);
  private final Timer homingTimer = new Timer();

  private LinearState linearState = LinearState.UNHOMED;
  // Boots RETRACTED and deploys only when the intake command is first issued. Once deployed it
  // stays out until explicitly retracted (or restored by the aggregation sweep).
  private double goalMeters = LINEAR_RETRACTED_POSITION_METERS;
  private int appliedCurrentLimitAmps = LINEAR_NORMAL_CURRENT_LIMIT_AMPS;
  private boolean wasEnabled = false;
  private boolean homingFailed = false;

  // Firing aggregation: sweep in slowly, then shutter between retracted and SHUTTER_OUT to herd
  // balls; the pre-aggregation posture is restored afterward
  private static final TrapezoidProfile.Constraints NORMAL_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          LINEAR_MAX_VELOCITY_METERS_PER_SEC, LINEAR_MAX_ACCELERATION_METERS_PER_SEC_SQ);
  private static final TrapezoidProfile.Constraints AGGREGATE_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          AGGREGATE_MAX_VELOCITY_METERS_PER_SEC, AGGREGATE_MAX_ACCELERATION_METERS_PER_SEC_SQ);
  private boolean aggregating = false;
  private boolean shutterOut = false;
  private double preAggregationGoalMeters = LINEAR_RETRACTED_POSITION_METERS;

  private final Alert homingFailedAlert =
      new Alert(
          "Intake homing timed out; position control disabled until re-enable.", AlertType.kError);
  private final Alert linearDisconnectedAlert =
      new Alert("Intake linear motor is disconnected.", AlertType.kError);
  private final Alert rollerDisconnectedAlert =
      new Alert("Intake roller motor is disconnected.", AlertType.kWarning);
  private final Alert linearTempAlert =
      new Alert("Intake linear motor is overheating.", AlertType.kWarning);
  private final Alert rollerTempAlert =
      new Alert("Intake roller motor is overheating.", AlertType.kWarning);

  public Intake(IntakeLinearIO linearIO, IntakeRollerIO rollerIO) {
    this.linearIO = linearIO;
    this.rollerIO = rollerIO;
    linearController.setTolerance(LINEAR_POSITION_TOLERANCE_METERS);
  }

  @Override
  public void periodic() {
    linearIO.updateInputs(linearInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Linear", linearInputs);
    Logger.processInputs("Intake/Rollers", rollerInputs);

    // Health monitoring
    linearDisconnectedAlert.set(!linearInputs.connected);
    rollerDisconnectedAlert.set(!rollerInputs.connected);
    linearTempAlert.set(linearInputs.tempCelsius > MOTOR_TEMP_WARNING_CELSIUS);
    rollerTempAlert.set(
        Math.max(rollerInputs.masterTempCelsius, rollerInputs.followerTempCelsius)
            > MOTOR_TEMP_WARNING_CELSIUS);
    homingFailedAlert.set(homingFailed);

    boolean enabled = DriverStation.isEnabled();
    boolean enableEdge = enabled && !wasEnabled;
    wasEnabled = enabled;
    if (enableEdge) {
      // Allow a homing retry after a disable/enable cycle
      homingFailed = false;
    }

    switch (linearState) {
      case UNHOMED -> {
        linearIO.setVoltage(0.0);
        // Automatic boot-up homing: start on the first enable (motors cannot move while disabled)
        if (enableEdge && !homingFailed) {
          startHoming();
        }
      }

      case HOMING -> {
        if (!enabled) {
          // Abort cleanly; retry on the next enable
          linearIO.setVoltage(0.0);
          linearState = LinearState.UNHOMED;
          break;
        }

        linearIO.setVoltage(HOMING_VOLTS);
        boolean stalled =
            stallDebouncer.calculate(
                linearInputs.currentAmps > HOMING_STALL_CURRENT_AMPS
                    && Math.abs(linearInputs.velocityMetersPerSec)
                        < HOMING_STALL_VELOCITY_METERS_PER_SEC);
        if (stalled) {
          // Found the hardstop: zero and hand over to position control. Stays retracted until
          // the intake command first deploys it.
          linearIO.setVoltage(0.0);
          linearIO.zeroPosition();
          goalMeters = LINEAR_RETRACTED_POSITION_METERS;
          linearController.reset(LINEAR_RETRACTED_POSITION_METERS);
          linearState = LinearState.RUNNING;
        } else if (homingTimer.hasElapsed(HOMING_TIMEOUT_SECS)) {
          linearIO.setVoltage(0.0);
          homingFailed = true;
          linearState = LinearState.UNHOMED;
        }
      }

      case RUNNING -> {
        if (!enabled) {
          // Track the measured position while disabled so re-enabling is bumpless
          linearController.reset(linearInputs.positionMeters);
          linearIO.setVoltage(0.0);
        } else {
          // Firing aggregation: once the current stroke settles, reverse direction (shutter)
          if (aggregating && linearController.atGoal()) {
            shutterOut = !shutterOut;
            setGoalMeters(
                shutterOut ? SHUTTER_OUT_POSITION_METERS : LINEAR_RETRACTED_POSITION_METERS);
          }
          double outputVolts = linearController.calculate(linearInputs.positionMeters, goalMeters);
          linearIO.setVoltage(MathUtil.clamp(outputVolts, -12.0, 12.0));
        }
      }
    }

    // NEO protection: drop the smart current limit heavily while holding against the retracted
    // hardstop, restore it while moving. Updated only on change (async config call).
    int desiredLimit =
        (linearState == LinearState.RUNNING && isHoldingAgainstHardstop())
            ? LINEAR_HOLDING_CURRENT_LIMIT_AMPS
            : (linearState == LinearState.HOMING
                ? LINEAR_HOMING_CURRENT_LIMIT_AMPS
                : LINEAR_NORMAL_CURRENT_LIMIT_AMPS);
    if (desiredLimit != appliedCurrentLimitAmps) {
      linearIO.setSmartCurrentLimit(desiredLimit);
      appliedCurrentLimitAmps = desiredLimit;
    }

    // Telemetry (state updates above are separate from this logging block)
    Logger.recordOutput("Intake/Linear/State", linearState.toString());
    Logger.recordOutput("Intake/Linear/Homed", linearState == LinearState.RUNNING);
    Logger.recordOutput("Intake/Linear/GoalMeters", goalMeters);
    Logger.recordOutput("Intake/Linear/SetpointMeters", linearController.getSetpoint().position);
    Logger.recordOutput("Intake/Linear/AtGoal", isAtGoal());
    Logger.recordOutput("Intake/Linear/CurrentLimitAmps", appliedCurrentLimitAmps);
  }

  private void startHoming() {
    linearState = LinearState.HOMING;
    homingTimer.restart();
    stallDebouncer.calculate(false); // Reset the debounce window
  }

  private boolean isHoldingAgainstHardstop() {
    return goalMeters <= LINEAR_MIN_POSITION_METERS + LINEAR_POSITION_TOLERANCE_METERS
        && linearController.atGoal();
  }

  /** Sets the linear articulation goal position (clamped to the mechanism's travel). */
  public void setGoalMeters(double positionMeters) {
    goalMeters =
        MathUtil.clamp(positionMeters, LINEAR_MIN_POSITION_METERS, LINEAR_MAX_POSITION_METERS);
  }

  /** Commands the intake to the extended (deployed) position. */
  public void extend() {
    setGoalMeters(LINEAR_EXTENDED_POSITION_METERS);
  }

  /** Commands the intake to the retracted (stowed) position. */
  public void retract() {
    setGoalMeters(LINEAR_RETRACTED_POSITION_METERS);
  }

  /** Returns true once boot-up homing has completed and position control is active. */
  public boolean isHomed() {
    return linearState == LinearState.RUNNING;
  }

  /** Returns true when the articulation has settled at its goal position. */
  public boolean isAtGoal() {
    return linearState == LinearState.RUNNING && linearController.atGoal();
  }

  /** Runs the roller bar at the specified voltage (follower tracks in hardware). */
  public void runRollers(double volts) {
    rollerIO.setVoltage(volts);
  }

  /** Stops the roller bar. */
  public void stopRollers() {
    rollerIO.setVoltage(0.0);
  }

  /**
   * Begins the firing aggregation sweep: the intake slowly comes fully in to herd balls, then
   * shutters between retracted and partially-out until {@link #stopAggregating()}. Idempotent —
   * safe to call every loop from a running command.
   */
  public void startAggregating() {
    if (!aggregating) {
      aggregating = true;
      shutterOut = false;
      preAggregationGoalMeters = goalMeters;
      linearController.setConstraints(AGGREGATE_CONSTRAINTS);
      setGoalMeters(LINEAR_RETRACTED_POSITION_METERS);
    }
  }

  /** Ends the aggregation sweep and restores the intake's pre-aggregation posture. */
  public void stopAggregating() {
    if (aggregating) {
      aggregating = false;
      linearController.setConstraints(NORMAL_CONSTRAINTS);
      setGoalMeters(preAggregationGoalMeters);
    }
  }

  /**
   * Command: ensure the intake is deployed (it lives extended, so usually a no-op) and run the
   * rollers while held. Releasing stops the rollers but leaves the intake OUT — it only comes in
   * for {@link #retractCommand()} or the firing aggregation sweep.
   */
  public Command intakeCommand() {
    return startEnd(
            () -> {
              extend();
              runRollers(ROLLER_INTAKE_VOLTS);
            },
            this::stopRollers)
        .withName("IntakeCollect");
  }

  /** Command: explicitly stow the intake (the only way it retracts and stays in). */
  public Command retractCommand() {
    return runOnce(this::retract).withName("IntakeRetract");
  }

  /** Command: run the rollers in reverse to eject a game piece. */
  public Command ejectCommand() {
    return startEnd(() -> runRollers(ROLLER_EJECT_VOLTS), this::stopRollers)
        .withName("IntakeEject");
  }

  /** Current linear articulation position in meters (used by sim and visualization). */
  public double getLinearPositionMeters() {
    return linearInputs.positionMeters;
  }

  /** Voltage currently applied to the roller master (used by the simulation manager). */
  public double getRollerAppliedVolts() {
    return rollerInputs.appliedVolts;
  }
}
