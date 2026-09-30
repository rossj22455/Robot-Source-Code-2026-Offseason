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
 * IO implementation for the roller NEOs on SparkMax controllers. The optional second motor (see
 * ROLLER_FOLLOWER_INSTALLED) is configured as a hardware-level inverted follower of the master, so
 * it tracks the master with no RIO involvement even if the RIO code stops running.
 */
public class IntakeRollerIOSparkMax implements IntakeRollerIO {
  // Protected so the sim subclass can wrap the devices in SparkMaxSims
  protected final SparkMax master = new SparkMax(ROLLER_MASTER_ID, MotorType.kBrushless);
  // Null when the second roller motor isn't installed (no controller on the CAN bus to talk to)
  protected final SparkMax follower =
      ROLLER_FOLLOWER_INSTALLED ? new SparkMax(ROLLER_FOLLOWER_ID, MotorType.kBrushless) : null;
  private final RelativeEncoder encoder = master.getEncoder();
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public IntakeRollerIOSparkMax() {
    var masterConfig = new SparkMaxConfig();
    masterConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .smartCurrentLimit(ROLLER_CURRENT_LIMIT_AMPS)
        .voltageCompensation(12.0);
    master.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (follower != null) {
      var followerConfig = new SparkMaxConfig();
      followerConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(ROLLER_CURRENT_LIMIT_AMPS)
          .voltageCompensation(12.0)
          .follow(ROLLER_MASTER_ID, true); // Hardware-level follower, inverted
      follower.configure(
          followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.connected = connectedDebouncer.calculate(!master.getFaults().can);
    inputs.velocityRpm = encoder.getVelocity();
    inputs.appliedVolts = master.getAppliedOutput() * master.getBusVoltage();
    inputs.masterCurrentAmps = master.getOutputCurrent();
    inputs.masterTempCelsius = master.getMotorTemperature();
    if (follower != null) {
      inputs.followerCurrentAmps = follower.getOutputCurrent();
      inputs.followerTempCelsius = follower.getMotorTemperature();
    }
  }

  @Override
  public void setVoltage(double volts) {
    master.setVoltage(volts);
  }
}
