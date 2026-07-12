// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Hardware-in-the-loop sim for the roller bar: the REAL {@link IntakeRollerIOSparkMax} runs
 * unchanged while REVLib's {@link SparkMaxSim} models the master controller against a NEO physics
 * model. Limitation: REV sim does not execute hardware following, so the follower's state is
 * mirrored from the master (its reported current is approximate).
 */
public class IntakeRollerIOSparkMaxSim extends IntakeRollerIOSparkMax {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private static final DCMotor MOTOR_MODEL = DCMotor.getNEO(1);
  // PLACEHOLDER sim-only value: single roller moment of inertia (direct drive assumed)
  private static final double SIM_MOI_KG_M2 = 0.0005;

  private final SparkMaxSim masterSim = new SparkMaxSim(master, MOTOR_MODEL);
  private final SparkMaxSim followerSim = new SparkMaxSim(follower, MOTOR_MODEL);
  private final DCMotorSim physics =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(MOTOR_MODEL, SIM_MOI_KG_M2, 1.0), MOTOR_MODEL);

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    double vbus = RobotController.getBatteryVoltage();

    physics.setInputVoltage(masterSim.getAppliedOutput() * vbus);
    physics.update(LOOP_PERIOD_SECS);

    // No conversion factors configured, so iterate() takes raw motor RPM. The hardware-inverted
    // follower is mirrored manually (REV sim limitation).
    masterSim.iterate(physics.getAngularVelocityRPM(), vbus, LOOP_PERIOD_SECS);
    followerSim.iterate(-physics.getAngularVelocityRPM(), vbus, LOOP_PERIOD_SECS);

    super.updateInputs(inputs);
  }
}
