// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.Constants.Shooter.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Hardware-in-the-loop sim for the drum: the REAL {@link ShooterDrumIOTalonFX} runs unchanged
 * (onboard velocity PID, follower opposition, current limits), while a {@link FlywheelSim} models
 * the drum physics and drives both Falcons' sim states. Config mistakes surface on desktop instead
 * of on the robot.
 */
public class ShooterDrumIOTalonFXSim extends ShooterDrumIOTalonFX {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private static final DCMotor GEARBOX = DCMotor.getFalcon500(2);

  private final FlywheelSim physics =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(GEARBOX, DRUM_MOI_KG_M2, DRUM_GEAR_RATIO), GEARBOX);

  private final TalonFXSimState masterSim = master.getSimState();
  private final TalonFXSimState followerSim = follower.getSimState();

  public ShooterDrumIOTalonFXSim() {
    // Both Falcons draw from the shared simulated battery, so drum spin-up sags the bus for
    // every other mechanism (and the drive), just like real hardware
    SimulatedBattery.addElectricalAppliances(() -> Amps.of(masterSim.getSupplyCurrent()));
    SimulatedBattery.addElectricalAppliances(() -> Amps.of(followerSim.getSupplyCurrent()));
  }

  @Override
  public void updateInputs(ShooterDrumIOInputs inputs) {
    // Phoenix sim requires the supply voltage every loop
    masterSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    followerSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

    // The onboard VelocityVoltage loop runs inside the Phoenix sim; read what it applied. The
    // two-motor gearbox model accounts for the follower's torque contribution.
    physics.setInputVoltage(masterSim.getMotorVoltage());
    physics.update(LOOP_PERIOD_SECS);

    // Reflect the mechanism back onto BOTH rotors; the opposed follower spins negative, so its
    // sim state (and current-limit behavior) matches real hardware
    double rotorRps = physics.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * DRUM_GEAR_RATIO;
    masterSim.setRotorVelocity(rotorRps);
    masterSim.addRotorPosition(rotorRps * LOOP_PERIOD_SECS);
    followerSim.setRotorVelocity(-rotorRps);
    followerSim.addRotorPosition(-rotorRps * LOOP_PERIOD_SECS);

    // Normal signal path — identical to the real robot
    super.updateInputs(inputs);
  }

  @Override
  public void simNotifyBallFired() {
    // Each ball robs the drum of ~4.93% of its surface speed (from the trajectory calculator's
    // spindown analysis); the onboard velocity PID then has to recover, exactly like real
    // hardware. This is what exercises the ready-tolerance interlock between shots.
    physics.setState(
        edu.wpi.first.math.VecBuilder.fill(physics.getAngularVelocityRadPerSec() * (1.0 - 0.0493)));
  }
}
