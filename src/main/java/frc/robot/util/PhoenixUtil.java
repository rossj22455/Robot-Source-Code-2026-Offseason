// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  // ---------------------------------------------------------------------------------------------
  // maple-sim <-> Phoenix 6 bridge helpers, taken from maple-sim's official
  // "AdvantageKit-TalonSwerveTemplate-MapleSim-Enhanced" template. These let maple-sim's physics
  // engine drive the real TalonFX device sim states each sub-tick, so the actual onboard control
  // loops (and device configs) run against the simulated mechanism.
  // ---------------------------------------------------------------------------------------------

  /**
   * Bridges a {@link TalonFX} into maple-sim: each simulation sub-tick, maple-sim provides the
   * mechanism state, this writes it into the device's sim state, and returns the motor voltage the
   * (simulated) device firmware decided to apply — closing the loop through the real onboard
   * control requests and configs.
   */
  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  /**
   * Same as {@link TalonFXMotorControllerSim}, but also feeds the mechanism state into a remote
   * CANcoder's sim state (for steer motors using Fused/Remote CANcoder feedback).
   */
  public static class TalonFXMotorControllerWithRemoteCancoderSim
      extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;

    public TalonFXMotorControllerWithRemoteCancoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.remoteCancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setRawPosition(mechanismAngle);
      remoteCancoderSimState.setVelocity(mechanismVelocity);
      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  /**
   * Synthesizes odometry sample timestamps matching maple-sim's sub-tick schedule, replacing the
   * timestamps that the Phoenix odometry thread would produce on a real robot.
   */
  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
    }
    return odometryTimeStamps;
  }

  /**
   * Swaps in sim-friendly steer gains and neutral inversions/offsets so the real module control
   * code works against maple-sim's physics. Returns the constants unchanged on a real robot. Gains
   * come from the maple-sim enhanced template and are tuned for its module model — the real robot's
   * tuned gains in TunerConstants are untouched.
   */
  public static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      regulateModuleConstantsForSimulation(
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
              moduleConstants) {
    if (RobotBase.isReal()) return moduleConstants;

    return moduleConstants
        .withEncoderOffset(0)
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withSteerMotorGains(
            new Slot0Configs()
                .withKP(70)
                .withKI(0)
                .withKD(4.5)
                .withKS(0)
                .withKV(1.91)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
        .withSteerMotorGearRatio(16.0)
        .withDriveFrictionVoltage(Volts.of(0.1))
        .withSteerFrictionVoltage(Volts.of(0.05))
        .withSteerInertia(KilogramSquareMeters.of(0.05));
  }
}
