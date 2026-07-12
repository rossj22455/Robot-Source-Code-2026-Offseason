// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Hardware-in-the-loop sim for the kicker: the REAL {@link ShooterKickerIOSparkMax} runs unchanged
 * while REVLib's {@link SparkMaxSim} models the controller (smart current limit, voltage comp,
 * conversion factors) against a {@link DCMotorSim} of the NEO + 3:1 gearbox.
 */
public class ShooterKickerIOSparkMaxSim extends ShooterKickerIOSparkMax {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private static final DCMotor GEARBOX = DCMotor.getNEO(1);
  // PLACEHOLDER sim-only value: kicker wheel moment of inertia at the output shaft
  private static final double SIM_MOI_KG_M2 = 0.0005;

  private final SparkMaxSim sparkSim = new SparkMaxSim(motor, GEARBOX);
  private final DCMotorSim physics =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(GEARBOX, SIM_MOI_KG_M2, KICKER_GEAR_RATIO), GEARBOX);

  @Override
  public void updateInputs(ShooterKickerIOInputs inputs) {
    double vbus = RobotController.getBatteryVoltage();

    // Step the physics from what the simulated controller applied last loop
    physics.setInputVoltage(sparkSim.getAppliedOutput() * vbus);
    physics.update(LOOP_PERIOD_SECS);

    // The velocity conversion factor reports output-shaft RPM, so iterate() takes the same units
    sparkSim.iterate(physics.getAngularVelocityRPM(), vbus, LOOP_PERIOD_SECS);

    super.updateInputs(inputs);
  }
}
