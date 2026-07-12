// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO for the shooter drum flywheel (2x Falcon 500, master + inverted hardware follower). Velocity
 * closed-loop runs on the motor controller (Phoenix 6 device-level PID), not on the RIO, for stable
 * RPM and fast recovery between shots.
 */
public interface ShooterDrumIO {
  @AutoLog
  public static class ShooterDrumIOInputs {
    public boolean connected = false;
    public boolean followerConnected = false;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double followerTempCelsius = 0.0;
  }

  public default void updateInputs(ShooterDrumIOInputs inputs) {}

  /** Commands the onboard velocity closed loop to the given drum RPM. */
  public default void setVelocityRpm(double rpm) {}

  /** Runs the drum open-loop at the specified voltage (characterization only). */
  public default void setVoltage(double volts) {}

  /** Stops the drum (coast). */
  public default void stop() {}

  /**
   * Simulation hook: applies the per-ball spindown impulse to the drum physics model when a shot is
   * fired. No-op on real hardware and in replay (the real drum feels the real ball).
   */
  public default void simNotifyBallFired() {}
}
