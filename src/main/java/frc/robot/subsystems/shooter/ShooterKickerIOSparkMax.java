// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.Constants.Shooter.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;

/** IO implementation for the kicker NEO on a SparkMax (REVLib, basic voltage commands). */
public class ShooterKickerIOSparkMax implements ShooterKickerIO {
  // Protected so the sim subclass can wrap the device in a SparkMaxSim
  protected final SparkMax motor = new SparkMax(KICKER_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public ShooterKickerIOSparkMax() {
    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(KICKER_CURRENT_LIMIT_AMPS)
        .voltageCompensation(12.0);
    // Report output-shaft (post 3:1 gearbox) velocity
    config.encoder.velocityConversionFactor(1.0 / KICKER_GEAR_RATIO);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterKickerIOInputs inputs) {
    inputs.connected = connectedDebouncer.calculate(!motor.getFaults().can);
    inputs.velocityRpm = encoder.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
