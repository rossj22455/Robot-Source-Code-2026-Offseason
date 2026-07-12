// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Hardware-in-the-loop sim for the indexer: the REAL {@link IndexerIOTalonFX} device code runs
 * unchanged (Brake mode, stator/supply limits) while a {@link DCMotorSim} models the belt drive and
 * feeds the Kraken's sim state.
 */
public class IndexerIOTalonFXSim extends IndexerIOTalonFX {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
  // PLACEHOLDER sim-only value: belt drive moment of inertia at the output shaft
  private static final double SIM_MOI_KG_M2 = 0.002;

  private final DCMotorSim physics =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(GEARBOX, SIM_MOI_KG_M2, INDEXER_GEAR_RATIO), GEARBOX);

  private final TalonFXSimState motorSim = motor.getSimState();

  public IndexerIOTalonFXSim() {
    // Draw from the shared simulated battery so the indexer contributes to bus sag
    SimulatedBattery.addElectricalAppliances(() -> Amps.of(motorSim.getSupplyCurrent()));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    motorSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

    physics.setInputVoltage(motorSim.getMotorVoltage());
    physics.update(LOOP_PERIOD_SECS);

    double rotorRps = physics.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * INDEXER_GEAR_RATIO;
    motorSim.setRotorVelocity(rotorRps);
    motorSim.addRotorPosition(rotorRps * LOOP_PERIOD_SECS);

    super.updateInputs(inputs);
  }
}
