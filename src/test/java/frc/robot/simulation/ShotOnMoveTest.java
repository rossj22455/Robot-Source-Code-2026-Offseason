// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.ShotOnMoveSolver;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * End-to-end validation of the shoot-on-the-move solver against maple-sim's real projectile physics
 * and hub scoring: launches Fuel with the solver's compensated heading/RPM while the (virtual)
 * chassis is translating, and asserts the arena actually scores it. The uncompensated control test
 * proves the compensation is doing real work — the same moving shot aimed straight at the hub
 * misses.
 *
 * <p>Projectile flight runs on the FPGA timer, so the tests pause wall-clock timing and step it in
 * lockstep with the arena.
 */
class ShotOnMoveTest {
  private static final Translation2d BLUE_HUB = new Translation2d(4.5974, 4.034536);
  // 2.6 m from the hub along -X: inside the tuned 1.7-4.3 m RPM table with margin for leads
  private static final Translation2d ROBOT_POSITION = new Translation2d(2.0, 4.034536);

  private Arena2026Rebuilt arena;

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
  }

  @AfterAll
  static void restoreTiming() {
    SimHooks.resumeTiming();
  }

  @BeforeEach
  void freshArena() {
    arena = new Arena2026Rebuilt(false);
    // Disable the alternating hub-active clock so every scored ball counts
    arena.setShouldRunClock(false);
    SimulatedArena.overrideInstance(arena);
  }

  /** Launches one Fuel using the solver's compensated aim while translating at fieldVelocity. */
  private void launchCompensated(Translation2d fieldVelocity) {
    var solution = ShotOnMoveSolver.solve(ROBOT_POSITION, fieldVelocity, BLUE_HUB);
    launch(fieldVelocity, solution.heading(), solution.distanceMeters());
  }

  private void launch(
      Translation2d fieldVelocity,
      edu.wpi.first.math.geometry.Rotation2d heading,
      double rangeMeters) {
    arena.addGamePieceProjectile(
        new RebuiltFuelOnFly(
            ROBOT_POSITION,
            ShooterConstants.BALL_EXIT_OFFSET,
            new ChassisSpeeds(fieldVelocity.getX(), fieldVelocity.getY(), 0.0),
            heading,
            Meters.of(ShooterConstants.BALL_EXIT_HEIGHT_METERS),
            MetersPerSecond.of(ShotOnMoveSolver.ballSpeedMps(rangeMeters)),
            Degrees.of(ShooterConstants.HOOD_ANGLE_DEG)));
  }

  /** Steps arena physics and the FPGA clock together long enough for the shot to resolve. */
  private void runSeconds(double seconds) {
    for (int i = 0; i < (int) (seconds / 0.02); i++) {
      arena.simulationPeriodic();
      SimHooks.stepTiming(0.02);
    }
  }

  private int blueScore() {
    return arena.getScore(true);
  }

  @Test
  void stationaryShotScores() {
    launchCompensated(Translation2d.kZero);
    runSeconds(4.0);
    assertEquals(1, blueScore(), "stationary compensated shot should score");
  }

  @Test
  void shotWhileDrivingTowardHubScores() {
    launchCompensated(new Translation2d(1.0, 0.0));
    runSeconds(4.0);
    assertEquals(1, blueScore(), "compensated shot while closing at 1 m/s should score");
  }

  @Test
  void shotWhileBackingAwayScores() {
    launchCompensated(new Translation2d(-1.0, 0.0));
    runSeconds(4.0);
    assertEquals(1, blueScore(), "compensated shot while retreating at 1 m/s should score");
  }

  @Test
  void shotWhileStrafingScores() {
    launchCompensated(new Translation2d(0.0, 1.5));
    runSeconds(4.0);
    assertEquals(1, blueScore(), "compensated shot while strafing at 1.5 m/s should score");
  }

  @Test
  void shotWhileMovingDiagonallyScores() {
    launchCompensated(new Translation2d(-0.8, 1.0));
    runSeconds(4.0);
    assertEquals(1, blueScore(), "compensated shot while moving diagonally should score");
  }

  @Test
  void uncompensatedStrafingShotMisses() {
    // Control: aim STRAIGHT at the hub (no lead) while strafing — the ~1.4 m of drift over the
    // flight must push the ball off the 0.6 m goal. If this ever starts scoring, the compensated
    // tests above prove nothing.
    Translation2d fieldVelocity = new Translation2d(0.0, 1.5);
    launch(
        fieldVelocity,
        BLUE_HUB.minus(ROBOT_POSITION).getAngle(),
        ROBOT_POSITION.getDistance(BLUE_HUB));
    runSeconds(4.0);
    assertEquals(0, blueScore(), "uncompensated strafing shot should miss");
  }
}
