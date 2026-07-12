// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.PhoenixUtil;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/**
 * Sim gyro backed by maple-sim's {@link GyroSimulation} (yaw drift and noise modeled). Pitch and
 * roll come from the synthetic terrain model (BumpSimulation), so ramp crossings tilt the robot and
 * the tilt-detection/recovery logic is exercisable in simulation.
 */
public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;
  private final Supplier<Rotation2d> pitchSupplier;
  private final Supplier<Rotation2d> rollSupplier;

  public GyroIOSim(
      GyroSimulation gyroSimulation,
      Supplier<Rotation2d> pitchSupplier,
      Supplier<Rotation2d> rollSupplier) {
    this.gyroSimulation = gyroSimulation;
    this.pitchSupplier = pitchSupplier;
    this.rollSupplier = rollSupplier;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);
    inputs.pitchPosition = pitchSupplier.get();
    inputs.rollPosition = rollSupplier.get();

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }
}
