// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Hardware-in-the-loop sim for the linear rack: the REAL {@link IntakeLinearIOSparkMax} runs
 * unchanged (conversion factors, smart current limits) while REVLib's {@link SparkMaxSim} models
 * the controller against a NEO + rack physics model with:
 *
 * <ul>
 *   <li>Gravity along the 23 degree rack incline (extension is uphill).
 *   <li>Hardstops at both ends of travel that zero velocity but NOT the applied voltage, so driving
 *       into a stop produces a realistic stall current (~8.8 A at the -1 V homing command) and the
 *       boot-up homing state machine completes exactly as on real hardware.
 *   <li>An unknown boot offset: the mechanism starts partially extended with the encoder reading
 *       zero, so homing has real work to do.
 * </ul>
 */
public class IntakeLinearIOSparkMaxSim extends IntakeLinearIOSparkMax {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private static final DCMotor GEARBOX = DCMotor.getNEO(1);
  // PLACEHOLDER sim-only values: reflected mechanism inertia, the unknown boot position, and
  // the equivalent voltage of static friction (brake mode + rack friction)
  private static final double SIM_MOI_KG_M2 = 0.002;
  private static final double SIM_BOOT_OFFSET_METERS = 0.05;
  private static final double STATIC_FRICTION_VOLTS = 0.5;
  // Meters of rack travel per output (pinion) rotation
  private static final double METERS_PER_OUTPUT_ROTATION =
      Math.PI * LINEAR_PINION_PITCH_DIAMETER_METERS;

  private final SparkMaxSim sparkSim = new SparkMaxSim(motor, GEARBOX);
  private final DCMotorSim physics =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(GEARBOX, SIM_MOI_KG_M2, LINEAR_GEAR_REDUCTION),
          GEARBOX);

  public IntakeLinearIOSparkMaxSim() {
    // Start partially extended; the SparkMax encoder still reads 0 (unknown offset, like boot)
    physics.setState(SIM_BOOT_OFFSET_METERS / METERS_PER_OUTPUT_ROTATION * 2.0 * Math.PI, 0.0);
  }

  @Override
  public void updateInputs(IntakeLinearIOInputs inputs) {
    double vbus = RobotController.getBatteryVoltage();

    // Gravity along the rack, reflected to an equivalent motor voltage (V = tau * R / Kt).
    // Extension (positive direction) is DOWN the 23 degree incline — the carriage slides out of
    // the robot toward the floor — so gravity assists extension and opposes retraction/homing.
    double gravityForceNewtons = CARRIAGE_MASS_KG * 9.81 * Math.sin(RACK_ANGLE_RAD);
    double motorTorqueNm =
        gravityForceNewtons * (LINEAR_PINION_PITCH_DIAMETER_METERS / 2.0) / LINEAR_GEAR_REDUCTION;
    double gravityVolts = motorTorqueNm * GEARBOX.rOhms / GEARBOX.KtNMPerAmp;

    double appliedVolts = sparkSim.getAppliedOutput() * vbus;
    double netVolts = appliedVolts + gravityVolts;

    // Stiction: brake mode + rack friction hold the carriage on real hardware, so small net
    // forces (like gravity alone while disabled) must not creep the mechanism in sim
    boolean nearlyStopped =
        Math.abs(physics.getAngularVelocityRPM()) / 60.0 * METERS_PER_OUTPUT_ROTATION < 0.01;
    if (nearlyStopped && Math.abs(netVolts) < STATIC_FRICTION_VOLTS) {
      physics.setState(physics.getAngularPositionRad(), 0.0);
      physics.setInputVoltage(0.0);
    } else {
      physics.setInputVoltage(MathUtil.clamp(netVolts, -vbus, vbus));
    }
    physics.update(LOOP_PERIOD_SECS);

    // Hardstops: freeze the mechanism but keep the controller's output untouched, so the motor
    // model reports true stall current while commanded into a stop
    double physicalPositionMeters =
        physics.getAngularPositionRotations() * METERS_PER_OUTPUT_ROTATION;
    if (physicalPositionMeters < 0.0) {
      physics.setState(0.0, 0.0);
    } else if (physicalPositionMeters > LINEAR_MAX_POSITION_METERS) {
      physics.setState(
          LINEAR_MAX_POSITION_METERS / METERS_PER_OUTPUT_ROTATION * 2.0 * Math.PI, 0.0);
    }

    // The velocity conversion factor reports meters/sec, so iterate() takes the same units; the
    // encoder position integrates from this velocity (frozen at a stop, like real hardware)
    double velocityMetersPerSec =
        physics.getAngularVelocityRPM() / 60.0 * METERS_PER_OUTPUT_ROTATION;
    sparkSim.iterate(velocityMetersPerSec, vbus, LOOP_PERIOD_SECS);

    super.updateInputs(inputs);

    // REV's sim reports signed motor current; real hardware reports magnitude. Normalize so the
    // homing stall detector sees hardware-equivalent values.
    inputs.currentAmps = Math.abs(inputs.currentAmps);
  }

  @Override
  public void zeroPosition() {
    super.zeroPosition();
    // Keep the controller sim's integrated position in lockstep with the encoder zero
    sparkSim.setPosition(0.0);
  }
}
