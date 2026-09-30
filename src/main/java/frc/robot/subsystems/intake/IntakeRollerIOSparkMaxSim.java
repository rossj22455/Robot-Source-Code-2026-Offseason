// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

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
  // Null when the follower isn't installed (mirrors the real IO)
  private final SparkMaxSim followerSim =
      follower != null ? new SparkMaxSim(follower, MOTOR_MODEL) : null;
  private final DCMotorSim physics =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(MOTOR_MODEL, SIM_MOI_KG_M2, 1.0), MOTOR_MODEL);

  public IntakeRollerIOSparkMaxSim() {
    // Supply current ~= stator current x duty cycle; both rollers contribute to battery sag
    SimulatedBattery.addElectricalAppliances(
        () -> Amps.of(Math.abs(masterSim.getMotorCurrent() * masterSim.getAppliedOutput())));
    if (followerSim != null) {
      SimulatedBattery.addElectricalAppliances(
          () -> Amps.of(Math.abs(followerSim.getMotorCurrent() * followerSim.getAppliedOutput())));
    }
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    double vbus = RobotController.getBatteryVoltage();

    physics.setInputVoltage(masterSim.getAppliedOutput() * vbus);
    physics.update(LOOP_PERIOD_SECS);

    // No conversion factors configured, so iterate() takes raw motor RPM. The hardware-inverted
    // follower is mirrored manually (REV sim limitation).
    masterSim.iterate(physics.getAngularVelocityRPM(), vbus, LOOP_PERIOD_SECS);
    if (followerSim != null) {
      followerSim.iterate(-physics.getAngularVelocityRPM(), vbus, LOOP_PERIOD_SECS);
    }

    super.updateInputs(inputs);
  }
}
