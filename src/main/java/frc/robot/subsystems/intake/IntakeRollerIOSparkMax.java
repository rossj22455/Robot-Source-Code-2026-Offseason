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
 * IO implementation for the dual-roller NEOs on SparkMax controllers. The follower is configured as
 * a hardware-level inverted follower of the master, so it tracks the master with no RIO involvement
 * even if the RIO code stops running.
 */
public class IntakeRollerIOSparkMax implements IntakeRollerIO {
  // Protected so the sim subclass can wrap the devices in SparkMaxSims
  protected final SparkMax master = new SparkMax(ROLLER_MASTER_ID, MotorType.kBrushless);
  protected final SparkMax follower = new SparkMax(ROLLER_FOLLOWER_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = master.getEncoder();
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public IntakeRollerIOSparkMax() {
    var masterConfig = new SparkMaxConfig();
    masterConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ROLLER_CURRENT_LIMIT_AMPS)
        .voltageCompensation(12.0);
    master.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var followerConfig = new SparkMaxConfig();
    followerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ROLLER_CURRENT_LIMIT_AMPS)
        .voltageCompensation(12.0)
        .follow(ROLLER_MASTER_ID, true); // Hardware-level follower, inverted
    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.connected = connectedDebouncer.calculate(!master.getFaults().can);
    inputs.velocityRpm = encoder.getVelocity();
    inputs.appliedVolts = master.getAppliedOutput() * master.getBusVoltage();
    inputs.masterCurrentAmps = master.getOutputCurrent();
    inputs.followerCurrentAmps = follower.getOutputCurrent();
    inputs.masterTempCelsius = master.getMotorTemperature();
    inputs.followerTempCelsius = follower.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    master.setVoltage(volts);
  }
}
