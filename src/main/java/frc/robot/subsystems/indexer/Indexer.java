// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static frc.robot.Constants.Indexer.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Indexer belt floor (1x Kraken X60) feeding game pieces from the intake to the shooter.
 *
 * <p>System coordination: the indexer and the shooter kicker act as one coordinated system. This
 * subsystem enforces the interlock at the lowest level — forward (toward-shooter) motion is refused
 * unless the shooter's {@code isReadyToShoot()} supplier returns true, so no command can
 * accidentally jam a ball into a drum that is not at speed. Reverse (unjam) motion is always
 * permitted.
 */
public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  // Bound to Shooter::isReadyToShoot in RobotContainer
  private final BooleanSupplier readyToShootSupplier;

  private double requestedVolts = 0.0;
  // Staging (gentle forward while the intake retracts) is exempt from the ready-to-shoot interlock
  private boolean staging = false;
  private boolean lastFeedBlocked = false;

  private final Alert disconnectedAlert =
      new Alert("Indexer motor is disconnected.", AlertType.kError);
  private final Alert tempAlert = new Alert("Indexer motor is overheating.", AlertType.kWarning);

  public Indexer(IndexerIO io, BooleanSupplier readyToShootSupplier) {
    this.io = io;
    this.readyToShootSupplier = readyToShootSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    // Health monitoring
    disconnectedAlert.set(!inputs.connected);
    tempAlert.set(inputs.tempCelsius > MOTOR_TEMP_WARNING_CELSIUS);

    // Interlock enforcement runs every loop, so if the shooter drops out of tolerance mid-feed
    // the belt stops immediately rather than continuing on a stale permission
    double outputVolts = requestedVolts;
    lastFeedBlocked = requestedVolts > 0.0 && !staging && !readyToShootSupplier.getAsBoolean();
    if (lastFeedBlocked) {
      outputVolts = 0.0;
    }
    io.setVoltage(outputVolts);

    // Telemetry
    Logger.recordOutput("Indexer/RequestedVolts", requestedVolts);
    Logger.recordOutput("Indexer/OutputVolts", outputVolts);
    Logger.recordOutput("Indexer/FeedBlockedByInterlock", lastFeedBlocked);
    Logger.recordOutput("Indexer/Staging", staging);
  }

  /**
   * Requests forward feed toward the shooter. Actual motion is gated by the shooter's
   * isReadyToShoot() interlock every loop; the request is held so feeding starts automatically the
   * moment the drum reaches speed.
   */
  public void feed() {
    requestedVolts = INDEXER_FEED_VOLTS;
    staging = false;
  }

  /**
   * Gently pulls balls forward into the robot while the intake retracts. The only forward motion
   * allowed without the drum at speed: it runs at the low {@code INDEXER_STAGE_VOLTS}, and the
   * braked kicker stops staged balls short of the drum.
   */
  public void stage() {
    requestedVolts = INDEXER_STAGE_VOLTS;
    staging = true;
  }

  /** Command: stage while running; stops the belt when it ends. */
  public Command stageCommand() {
    return startEnd(this::stage, this::stop).withName("IndexerStage");
  }

  /** Runs the belt in reverse, away from the shooter (always permitted, for clearing jams). */
  public void reverse() {
    requestedVolts = INDEXER_REVERSE_VOLTS;
    staging = false;
  }

  /** Stops the belt. */
  public void stop() {
    requestedVolts = 0.0;
    staging = false;
  }

  /** Returns true when a forward feed request is currently being blocked by the interlock. */
  public boolean isFeedBlocked() {
    return lastFeedBlocked;
  }

  /** Voltage currently applied to the belt motor (used by the simulation manager). */
  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  /** Belt-drive velocity in RPM (used by the 3D visualizer). */
  public double getVelocityRpm() {
    return inputs.velocityRpm;
  }
}
