// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO for the intake linear rack articulation motor. The IO layer only feeds raw sensor data up and
 * accepts voltage commands down — all position control runs on the RIO in {@link Intake}.
 */
public interface IntakeLinearIO {
  @AutoLog
  public static class IntakeLinearIOInputs {
    public boolean connected = false;
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(IntakeLinearIOInputs inputs) {}

  /** Runs the motor at the specified open-loop voltage. */
  public default void setVoltage(double volts) {}

  /**
   * Updates the smart current limit (used to drop the limit while holding against the hardstop).
   */
  public default void setSmartCurrentLimit(int amps) {}

  /** Zeros the encoder at the current position (called when homing finds the hardstop). */
  public default void zeroPosition() {}

  /** Sets the idle mode: brake when true, coast when false. */
  public default void setBrakeMode(boolean brake) {}
}
