// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.EventLog;
import java.nio.file.Files;
import java.nio.file.Path;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private final Alert usbLogMissingAlert =
      new Alert(
          "No USB stick at /U — this session is NOT being logged. Insert a FAT32 USB stick.",
          AlertType.kError);

  // Last-seen states for event logging (events are recorded only on change)
  private boolean lastBrownedOut = false;
  private boolean lastDsAttached = false;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs"). The roboRIO only mounts a
        // FAT32-formatted stick at /U; without one nothing is recorded, so say so loudly.
        usbLogMissingAlert.set(!Files.isDirectory(Path.of("/U")));
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Event log: every command start/end/interrupt (named auto commands, paths, button commands)
    CommandScheduler.getInstance()
        .onCommandInitialize(command -> EventLog.log("START " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> EventLog.log("END " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (command, interruptor) ->
                EventLog.log(
                    "INTERRUPTED "
                        + command.getName()
                        + interruptor.map(by -> " by " + by.getName()).orElse("")));

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Transition events (cheap: two boolean reads, logged only when they change)
    boolean brownedOut = RobotController.isBrownedOut();
    if (brownedOut != lastBrownedOut) {
      EventLog.log(
          brownedOut
              ? String.format("BROWNOUT (battery %.2f V)", RobotController.getBatteryVoltage())
              : "Brownout recovered");
      lastBrownedOut = brownedOut;
    }
    boolean dsAttached = DriverStation.isDSAttached();
    if (dsAttached != lastDsAttached) {
      EventLog.log(dsAttached ? "Driver Station connected" : "Driver Station connection LOST");
      lastDsAttached = dsAttached;
    }

    // Publish 3D component poses for AdvantageScope from the freshly-updated subsystem state
    robotContainer.updateVisualization();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    EventLog.log("Mode: DISABLED");
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.resetSimulationField();
    autonomousCommand = robotContainer.getAutonomousCommand();
    EventLog.log(
        "Mode: AUTONOMOUS ("
            + (autonomousCommand != null ? autonomousCommand.getName() : "no auto selected")
            + ", "
            + DriverStation.getAlliance().map(Enum::name).orElse("no alliance")
            + ")");

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    EventLog.log("Mode: TELEOP");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // Home the intake slide automatically on entering test mode
    CommandScheduler.getInstance().schedule(robotContainer.getTestCommand());
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // Steps the maple-sim arena (drivetrain physics, game pieces, scoring). REPLAY also runs on
    // desktop, but the simulation manager is only constructed in SIM mode, so replay stays clean.
    robotContainer.updateSimulation();

    // Automated self-test support (SIM_TEST=1): attach a virtual driver station and enable
    // teleop a few seconds after boot so the scripted test sequence in RobotContainer can run
    // headless (no sim GUI required)
    if (Constants.currentMode == Constants.Mode.SIM
        && "1".equals(System.getenv("SIM_TEST"))
        && !DriverStationSim.getEnabled()
        && Timer.getFPGATimestamp() > 3.0) {
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
    }
  }
}
