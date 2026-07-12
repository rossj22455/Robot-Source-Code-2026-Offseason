// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** IO for the shooter kicker (1x NEO on a SparkMax through a 3:1 gearbox, voltage control only). */
public interface ShooterKickerIO {
  @AutoLog
  public static class ShooterKickerIOInputs {
    public boolean connected = false;
    public double velocityRpm = 0.0; // Output (post-gearbox) RPM
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(ShooterKickerIOInputs inputs) {}

  /** Runs the kicker at the specified open-loop voltage. */
  public default void setVoltage(double volts) {}
}
