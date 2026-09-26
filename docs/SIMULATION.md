# Simulation Guide

High-fidelity desktop simulation of the full robot: maple-sim field physics (2026 REBUILT game
with Fuel), Phoenix 6 / REVLib device-level simulation, and 3D component broadcasting for
AdvantageScope.

## Running the simulation

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'   # use the WPILib JDK
$env:SIM_GUI = '1'                                      # open the sim + Driver Station windows
./gradlew simulateJava
```

Connect AdvantageScope to `localhost` (NT4). Use the sim GUI (or a real gamepad on port 0) to
enable and drive. The GUI stays disabled by default for replay compatibility; set `SIM_GUI=1` to
open it.

### Automated self-test

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'; $env:SIM_TEST = '1'
./gradlew simulateJava
```

With `SIM_TEST=1` the robot attaches a virtual driver station, enables teleop, and runs a
scripted sequence (homing → intake deploy while driving → full firing chain), printing
`[SIMTEST]` markers to the console. Expected pass looks like:

```
[SIMTEST] homing: homed=true pos=0.0000 m
[SIMTEST] deployed: pos=0.2918 m atGoal=true
[SIMTEST] shoot: ready=true drumRpm=1814 distValid=true dist=2.63 m shots=2
[SIMTEST] SEQUENCE COMPLETE: shots=2 homed=true retractedPos=0.0121 m
```

(The test preloads exactly 2 Fuel, so `shots=2` means everything fired. Exact RPM/distance vary
slightly run to run — the robot aims itself before shooting. The deployed position can sit
slightly below the goal from gravity sag on the 23° rack with the real 4.93 kg carriage; still
within the at-goal tolerance.)

Run this after any change to the mechanism IO, constants, or interlock logic — it exercises the
real device configs, the onboard drum velocity PID, the homing state machine, the vision
confidence gate, and game-piece flow end to end.

## Architecture

Every mechanism's **real** hardware IO class runs unchanged in sim. A sim subclass (e.g.
`ShooterDrumIOTalonFXSim extends ShooterDrumIOTalonFX`) steps a WPILib physics model and feeds it
into the device's vendor sim state (`TalonFXSimState` / `SparkMaxSim`), so the actual device
configs, onboard closed-loop control, follower setup, and current limits are exercised on the
desktop. Config mistakes surface in sim instead of on the robot.

- **Swerve + field physics**: `simulation/TerrainAwareSwerveDriveSimulation` (extends maple-sim's
  `SwerveDriveSimulation`) in `Arena2026Rebuilt` (rigid-body chassis, field obstacles, Fuel game
  pieces, hub scoring). Wired in `RobotContainer`'s SIM branch; module/gyro bridges in
  `ModuleIOTalonFXSim`, `GyroIOSim`, and `util/PhoenixUtil`.
- **3D bump dynamics**: maple-sim is 2D, so `TerrainAwareSwerveDriveSimulation` adds three
  out-of-plane degrees of freedom (heave/pitch/roll) integrated at the physics sub-tick rate:
  each wheel is a stiff spring-damper contact sampled against the `simulation/FieldTerrain`
  height field (the hub ramps). Coupling is two-way — wheel contacts transmit real down-slope
  forces into the planar body (climbing a ramp slows the robot; straddling an edge yaws it), and
  the live per-wheel normal forces feed each module's grip limit (weight transfer under
  accel/braking, unloaded wheels lose traction). The sim gyro reads the resulting pitch/roll, so
  tilt detection/recovery sees realistic dynamic transients, not a static lookup.
- **Battery sag**: all drive motors (automatic via maple-sim) and every mechanism sim (drum,
  kicker, indexer, intake linear + rollers — registered in their sim IO constructors) draw from
  maple-sim's shared `SimulatedBattery`. The loaded bus voltage feeds back into every device sim
  and `RobotController.getBatteryVoltage()`, so a drum spin-up during a hard launch measurably
  slows the drivetrain, exactly like a real match.
- **Mechanisms**: `ShooterDrumIOTalonFXSim` (FlywheelSim → both Falcons, follower opposed),
  `IndexerIOTalonFXSim`, `ShooterKickerIOSparkMaxSim`, `IntakeLinearIOSparkMaxSim` (gravity on
  the 23° rack, hardstops with realistic stall current so **homing completes in sim**),
  `IntakeRollerIOSparkMaxSim`.
- **Game-piece lifecycle**: `simulation/SimulationManager` (SIM-only) binds the maple-sim intake
  volume to the real intake's extension + roller state, converts kicker feeds into launched
  `RebuiltFuelOnFly` projectiles at the drum's surface speed, and steps the arena.
- **3D visualization**: `util/RobotVisualizer` (all modes, works in replay) publishes component
  poses every loop from logged subsystem state.

## Published keys for AdvantageScope

| Key | Type | Frame | Notes |
| --- | --- | --- | --- |
| `/RealOutputs/Components` | `Pose3d[4]` | Robot | `[0]` intake carriage, `[1]` shooter drum, `[2]` indexer roller, `[3]` kicker roller |
| `/RealOutputs/FieldSimulation/GamePieces` | `Pose3d[]` | Field | All Fuel on the field / in flight |
| `/RealOutputs/FieldSimulation/SimulatedRobotPose` | `Pose2d` | Field | Ground truth (compare against `/RealOutputs/Odometry/Robot`) |
| `/RealOutputs/FieldSimulation/SimulatedRobotPose3d` | `Pose3d` | Field | Ground truth with terrain heave/pitch/roll — bind the 3D robot to this to see it climb the ramps |
| `/RealOutputs/FieldSimulation/Terrain/*` | mixed | — | `PitchDeg`/`RollDeg`/`HeaveMeters`, `WheelNormalForcesN[4]` (0 = wheel airborne), `OnRamp` |
| `/RealOutputs/FieldSimulation/Battery/*` | `double` | — | `VoltageVolts` (loaded bus), `TotalCurrentAmps` (all registered appliances) |
| `/RealOutputs/FieldSimulation/ShotTrajectory` | `Pose3d[]` | Field | Predicted arc of the most recent shot |
| `/RealOutputs/FieldSimulation/BallsInRobot` | `Pose3d[]` | Field | Fuel riding the intake→kicker path inside the robot (render as game pieces) |
| `/RealOutputs/FieldSimulation/PiecesInRobot` | `int` | — | Fuel currently held |
| `/RealOutputs/FieldSimulation/ShotsFired` | `int` | — | Cumulative launches |
| `/RealOutputs/FieldSimulation/BlueScore`, `RedScore` | `int` | — | Arena hub scoring |
| `/SmartDashboard/MapleSim/Goals/*` | `Pose3d` | Field | Hub positions (published by maple-sim) |

Units are meters/radians throughout. Vision tag/robot poses are under `/RealOutputs/Vision/*`
(from the existing template).

## Zeroed-pose convention (CAD alignment)

Convention (matches the AdvantageScope custom-assets guide / Littleton video):

1. **JSON = mesh normalization.** Each component's `zeroedPosition`/`zeroedRotations` in the
   robot asset JSON only bring the exported CAD mesh onto the grid origin, cleanly axis-aligned,
   with its reference point (rotation axis / carriage datum) at 0,0,0. Calibrate this while the
   published pose is identity (mechanism homed, `ZEROED_*` still zero).
2. **Code = mount + motion.** The `ZEROED_*` constants in `RobotVisualizer` are each component's
   mounted position in the ROBOT frame (x forward, y left, z up, meters, from robot center at
   floor level). The published pose = mount + motion, which is what carries the part from the
   grid origin onto the robot.
3. Motion on top of the mount:
   - Intake carriage: translates along `(±cos 23°, 0, sin 23°)` by the linear position, i.e.
     `(0.269, 0, 0.114)` at full travel (`EXTENSION_DIRECTION_X` in `RobotVisualizer` flips the
     direction if the model slides the wrong way).
   - Drum / indexer / kicker: spin about the robot Y axis (change the `Rotation3d` axis in
     `RobotVisualizer` if a CAD axis differs).

## Constants that still need real values

| Constant | Where | Status |
| --- | --- | --- |
| `DRUM_MOI_KG_M2`, `DRUM_RADIUS_METERS` | `ShooterConstants` | **Done** — CAD: 0.00467 kg·m² (Lxx), 2" radius (4" wheels) |
| `DISTANCE_TO_RPM_MAP` | `ShooterConstants` | **Done** — trajectory calculator (64°, η=0.9); verify with real shots. No solution below ~1.7 m |
| Drum PID/FF | `ShooterConstants` | Sim-workable starting gains — retune on hardware |
| `robotToCamera0..2` | `VisionConstants` | Completed |
| Camera names | `VisionConstants` | Must match PhotonVision UI — keep as-is for now |
| `CARRIAGE_MASS_KG` | `IntakeConstants` | **Done** — CAD 10.863 lb = 4.93 kg (excludes fasteners, slightly light) |
| Linear PID + profile, roller/kicker/indexer volts | `IntakeConstants`, etc. | Sim-workable placeholders — tune on hardware |
| `ZEROED_*` mounts, `EXTENSION_DIRECTION_X` | `RobotVisualizer` | Fill from CAD |
| Intake width, shot angle/efficiency/height | `SimulationManager` | **Done** — 23.66", 64°, η=0.9, 17.973" exit height |
| Hopper capacity, ball path start/end, transport speed, shooter exit offset | `SimulationManager` | Placeholders — set capacity + path points from CAD |
| Bumper size | `Drive.mapleSimConfig` | **Done** — 36.5" × 33.5" (intake side is the 33.5" front edge) |

Verified real values already encoded: drum ratio 15:12 (12t→105t→15t, the 105t idler cancels —
**assumes one pulley per shaft**), indexer 70:16, intake travel 0.29207 m @ 23°, intake gearing
16t→62t + 10DP 10t pinion, hub centers (4.5974, 4.0345) / (11.938, 4.0345).

**CAN IDs**: mechanism IDs are placeholder assignments 40–46 (drum 40/41, kicker 42, indexer 43,
linear 44, rollers 45/46). The original placeholders (93–99) were outside the legal FRC CAN
device ID range of 0–62 and would have been silently non-functional on the real bus (they were in
simulation, which is how this was caught). Keep any reassignment within 0–62.

## Known sim limitations

- Bump simulation is terrain *dynamics*, not full 3D collision: the arena's ramp colliders are
  disabled and `TerrainAwareSwerveDriveSimulation` models the ramps as a height field with real
  chassis physics (slope forces, per-wheel loads, dynamic pitch/roll on the gyro). What it cannot
  do: bumpers hitting the ramp side walls (driving across a lateral ramp edge steps the terrain
  instead of colliding), game pieces interacting with the slopes (Fuel stays in the 2D plane),
  or rolling the robot over. Default ramp rise (~5.3°) stays under the 10° tilt threshold —
  raise `FieldTerrain.RAMP_HEIGHT_METERS` above ~0.38 m to force tilt-recovery events.
- Contact stiffness/damping, CG height, and pitch/roll MOI in
  `TerrainAwareSwerveDriveSimulation` are physically-plausible PLACEHOLDER values — replace with
  CAD/measured numbers when available.
- REV hardware following isn't simulated; the roller follower is mirrored manually.
- Drive gains in sim are maple-sim's regulated values, not the robot's tuned gains
  (`PhoenixUtil.regulateModuleConstantsForSimulation`), so drive "feel" differs slightly.
- Shot physics (efficiency, hood angle, exit height) are placeholders until real shot data exists.
- The intake sim collects any Fuel touching the collision volume; it does not model the roller
  surface speed or piece jams.
- Ball transport is a straight-line path at constant belt speed (no per-ball physics inside the
  robot). Each launch applies the calculator's 4.93% drum spindown, so the ready-tolerance
  interlock and PID recovery between shots are exercised realistically.

## Replay

REPLAY mode is untouched: sim classes are only constructed in `Constants.Mode.SIM`, and
`RobotVisualizer` runs from logged inputs, so component poses re-render during replay.
