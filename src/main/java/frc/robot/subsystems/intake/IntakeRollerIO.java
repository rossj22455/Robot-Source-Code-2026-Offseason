// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO for the intake roller bar. Two NEOs drive the roller (one per end); the second motor is a
 * hardware-level inverted follower, so only the master is commanded and monitored here (plus the
 * follower's health signals).
 */
public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean connected = false;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double masterCurrentAmps = 0.0;
    public double followerCurrentAmps = 0.0;
    public double masterTempCelsius = 0.0;
    public double followerTempCelsius = 0.0;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  /** Runs the roller at the specified open-loop voltage. */
  public default void setVoltage(double volts) {}
}
