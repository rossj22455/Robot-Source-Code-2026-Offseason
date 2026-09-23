// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;

/**
 * IO implementation for the linear articulation NEO on a SparkMax. Feeds raw encoder data up and
 * accepts voltage commands down; no onboard closed-loop control is used (RIO-side PID only).
 */
public class IntakeLinearIOSparkMax implements IntakeLinearIO {
  // Protected so the sim subclass can wrap the device in a SparkMaxSim
  protected final SparkMax motor = new SparkMax(LINEAR_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  // Reused for non-blocking current limit updates; only the current limit field changes at
  // runtime, so other settings are never touched after the initial blocking configuration.
  private final SparkMaxConfig currentLimitConfig = new SparkMaxConfig();

  // Same idea for the idle mode, which flips between brake (enabled) and coast (disabled)
  private final SparkMaxConfig idleModeConfig = new SparkMaxConfig();

  public IntakeLinearIOSparkMax() {
    this(LINEAR_MOTOR_INVERTED);
  }

  /**
   * @param inverted flips both the motor output and the encoder so positive always means "extend".
   *     The sim subclass passes false because its physics model is already built positive-extend.
   */
  protected IntakeLinearIOSparkMax(boolean inverted) {
    var config = new SparkMaxConfig();
    config
        .inverted(inverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(LINEAR_NORMAL_CURRENT_LIMIT_AMPS)
        .voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(LINEAR_METERS_PER_MOTOR_ROTATION)
        .velocityConversionFactor(LINEAR_METERS_PER_MOTOR_ROTATION / 60.0);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeLinearIOInputs inputs) {
    inputs.connected = connectedDebouncer.calculate(!motor.getFaults().can);
    inputs.positionMeters = encoder.getPosition();
    inputs.velocityMetersPerSec = encoder.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setSmartCurrentLimit(int amps) {
    // Async so the 20ms loop is never blocked; called only on state transitions, not per-loop
    currentLimitConfig.smartCurrentLimit(amps);
    motor.configureAsync(
        currentLimitConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void zeroPosition() {
    encoder.setPosition(0.0);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    // Async so the 20ms loop is never blocked; called only on enable/disable transitions. Not
    // persisted, so the flashed default (brake) is what the motor boots with.
    idleModeConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configureAsync(
        idleModeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
