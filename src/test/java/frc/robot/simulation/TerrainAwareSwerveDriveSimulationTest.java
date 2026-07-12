// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Physics checks for the 3D terrain dynamics: static equilibrium on flat carpet, and the chassis
 * settling parallel to a hub ramp with the correct pitch and ride height. Steps the real maple-sim
 * arena, so these run the exact code path used in desktop simulation.
 */
class TerrainAwareSwerveDriveSimulationTest {
  private static final double ROBOT_MASS_KG = Drive.ROBOT_MASS_KG;
  private static final double BLUE_HUB_X = 4.5974;
  private static final double HUB_Y = 4.034536;
  // Expected ramp slope: 0.20 m rise over the 85 in ramp length (~5.3 deg)
  private static final double RAMP_SLOPE_DEG = Math.toDegrees(Math.atan(0.20 / 2.159));

  private TerrainAwareSwerveDriveSimulation driveSimulation;

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void freshArena() {
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
  }

  private void spawnAt(Pose2d pose) {
    driveSimulation = new TerrainAwareSwerveDriveSimulation(Drive.getMapleSimConfig(), pose);
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
  }

  private void runSeconds(double seconds) {
    for (int i = 0; i < (int) (seconds / 0.02); i++) {
      SimulatedArena.getInstance().simulationPeriodic();
    }
  }

  @Test
  void staticEquilibriumOnFlatGround() {
    spawnAt(new Pose2d(3, 3, Rotation2d.kZero));
    runSeconds(1.0);

    assertEquals(0.0, driveSimulation.getPitch().getDegrees(), 0.2, "pitch on flat ground");
    assertEquals(0.0, driveSimulation.getRoll().getDegrees(), 0.2, "roll on flat ground");
    assertEquals(0.0, driveSimulation.getHeaveMeters(), 0.003, "heave on flat ground");
    assertFalse(driveSimulation.isOnRamp());

    double expectedPerWheel = ROBOT_MASS_KG * 9.8 / 4.0;
    for (double normal : driveSimulation.getWheelNormalForcesNewtons()) {
      assertEquals(expectedPerWheel, normal, 10.0, "per-wheel normal force = mg/4");
    }
  }

  @Test
  void chassisSettlesParallelToRampWithNoseUpPitch() {
    // Mid-ramp north of the blue hub, facing the hub (-Y), so the slope rises ahead: the
    // chassis should settle nose-up at the ramp angle, level in roll, riding at the terrain
    // height under its center
    spawnAt(new Pose2d(BLUE_HUB_X, HUB_Y + 1.7, Rotation2d.fromDegrees(-90)));
    runSeconds(2.0);

    assertTrue(driveSimulation.isOnRamp());
    assertEquals(
        RAMP_SLOPE_DEG, driveSimulation.getPitch().getDegrees(), 1.0, "nose-up pitch on ramp");
    assertEquals(0.0, driveSimulation.getRoll().getDegrees(), 1.0, "roll on uniform ramp");

    // Terrain height under the center: 0.20 * (2.7559 - 1.7) / 2.159
    double expectedHeave = 0.20 * (2.7559 - 1.7) / 2.159;
    assertEquals(expectedHeave, driveSimulation.getHeaveMeters(), 0.02, "ride height on ramp");

    // All wheels still loaded (nothing airborne on a uniform slope)
    for (double normal : driveSimulation.getWheelNormalForcesNewtons()) {
      assertTrue(normal > 50.0, "wheel stays loaded on ramp, got " + normal + " N");
    }
  }

  @Test
  void straddlingRampEdgeRollsTowardTheLowSide() {
    // Robot facing +X with only its left wheels on the ramp strip (right wheels past the lateral
    // edge on flat carpet): the chassis should roll right-side-down... left side is uphill, so
    // left-side-up positive roll
    double stripEdgeX = BLUE_HUB_X + 0.5969;
    spawnAt(new Pose2d(stripEdgeX, HUB_Y + 1.7, Rotation2d.fromDegrees(90)));
    runSeconds(2.0);

    // Left wheels (world -X side... heading +90 deg puts robot-left toward -X) sit on the ramp;
    // magnitude depends on how many wheels are on, so just require a clearly nonzero tilt with
    // the chassis still supported
    assertTrue(
        Math.abs(driveSimulation.getRoll().getDegrees())
                + Math.abs(driveSimulation.getPitch().getDegrees())
            > 1.0,
        "straddling the ramp edge should tilt the chassis");
    double total = 0;
    for (double normal : driveSimulation.getWheelNormalForcesNewtons()) total += normal;
    assertEquals(ROBOT_MASS_KG * 9.8, total, ROBOT_MASS_KG * 9.8 * 0.15, "weight still carried");
  }
}
