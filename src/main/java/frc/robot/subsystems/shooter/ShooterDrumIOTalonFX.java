// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/**
 * IO implementation for the drum flywheel: 2x Falcon 500 via Phoenix 6. The second Falcon is a
 * strict hardware-level follower running inverted (opposed direction), so it mirrors the master
 * even if the RIO code stops. Velocity closed-loop runs onboard the master (device-level PID).
 */
public class ShooterDrumIOTalonFX implements ShooterDrumIO {
  // Protected so the sim subclass can reach the device sim states
  protected final TalonFX master = new TalonFX(DRUM_MASTER_ID);
  protected final TalonFX follower = new TalonFX(DRUM_FOLLOWER_ID);

  // Control requests are cached and reused to avoid per-loop allocation
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final StatusSignal<AngularVelocity> velocity = master.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = master.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrent = master.getSupplyCurrent();
  private final StatusSignal<Current> statorCurrent = master.getStatorCurrent();
  private final StatusSignal<Temperature> temp = master.getDeviceTemp();
  private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent();
  private final StatusSignal<Temperature> followerTemp = follower.getDeviceTemp();

  public ShooterDrumIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Never brake a flywheel
    config.Slot0.kP = DRUM_P;
    config.Slot0.kI = DRUM_I;
    config.Slot0.kD = DRUM_D;
    config.Slot0.kS = DRUM_S;
    config.Slot0.kV = DRUM_V;
    config.CurrentLimits.StatorCurrentLimit = DRUM_STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = DRUM_SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    PhoenixUtil.tryUntilOk(5, () -> master.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config, 0.25));

    // Strict hardware-level follower, running inverted relative to the master (the two Falcons
    // face opposite sides of the drum)
    follower.setControl(new Follower(DRUM_MASTER_ID, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        velocity,
        appliedVolts,
        supplyCurrent,
        statorCurrent,
        temp,
        followerStatorCurrent,
        followerTemp);
    master.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterDrumIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(velocity, appliedVolts, supplyCurrent, statorCurrent, temp)
            .equals(StatusCode.OK);
    inputs.followerConnected =
        BaseStatusSignal.refreshAll(followerStatorCurrent, followerTemp).equals(StatusCode.OK);
    inputs.velocityRpm = velocity.getValueAsDouble() * 60.0 / DRUM_GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.followerStatorCurrentAmps = followerStatorCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.followerTempCelsius = followerTemp.getValueAsDouble();
  }

  @Override
  public void setVelocityRpm(double rpm) {
    // Drum RPM -> motor rotations per second
    master.setControl(velocityRequest.withVelocity(rpm / 60.0 * DRUM_GEAR_RATIO));
  }

  @Override
  public void setVoltage(double volts) {
    master.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    master.setControl(neutralRequest);
  }
}
