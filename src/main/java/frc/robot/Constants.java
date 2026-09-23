// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Central constants for the robot. Runtime mode lives at the top; every field-tunable value for the
 * mechanisms (CAN IDs, RPMs, voltages, current limits, setpoints, gains) is gathered into the
 * nested classes below so there is one place to find and edit them. Drive/vision constants stay in
 * their own files (drive is CTRE-generated; vision is camera-specific).
 *
 * <p>The mode is always "real" when running on a roboRIO. Change the value of "simMode" to switch
 * between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /**
   * Intake: linear rack articulation (1x NEO) and dual-roller bar (2x NEO, master + inverted
   * hardware follower). All tuning gains and physical setpoints are PLACEHOLDERS; protection limits
   * are conservative starting values to be validated on the real mechanism.
   */
  public static final class Intake {
    // CAN IDs — PLACEHOLDER assignments, but note the legal FRC CAN device ID range is 0-62
    public static final int LINEAR_MOTOR_ID = 33;
    public static final int ROLLER_MASTER_ID = 45;
    public static final int ROLLER_FOLLOWER_ID = 46;

    // The slide motor's raw direction is negative = extend (measured: fully extended read about
    // -0.280 m). Inverting it makes positive = extend everywhere in code, matching the setpoints.
    public static final boolean LINEAR_MOTOR_INVERTED = true;

    // Linear mechanism gearing: 16t gear drives a 62t gear, which drives a 10 DP, 10t pinion on the
    // rack. Pinion pitch diameter = 10 teeth / 10 DP = 1.0 in, so the rack travels PI * 1.0 in per
    // pinion revolution. These values are derived from the mechanism, not tuned.
    public static final double LINEAR_GEAR_REDUCTION =
        62.0 / 16.0; // Motor rotations per pinion rot
    public static final double LINEAR_PINION_PITCH_DIAMETER_METERS = Units.inchesToMeters(1.0);
    public static final double LINEAR_METERS_PER_MOTOR_ROTATION =
        (1.0 / LINEAR_GEAR_REDUCTION) * Math.PI * LINEAR_PINION_PITCH_DIAMETER_METERS;

    // Linear travel — the homing sequence defines 0.0 at the retracted hardstop. Full travel is
    // 11.498946 in hardstop-to-hardstop (from CAD). The working setpoints sit slightly inside the
    // hardstops so position control never slams the mechanical limits.
    public static final double LINEAR_MIN_POSITION_METERS = 0.0;
    public static final double LINEAR_MAX_POSITION_METERS = Units.inchesToMeters(11.498946);
    public static final double LINEAR_RETRACTED_POSITION_METERS = 0.003;
    public static final double LINEAR_EXTENDED_POSITION_METERS = LINEAR_MAX_POSITION_METERS - 0.005;

    // The rack extends out of the robot on a 23 degree incline from horizontal (from CAD). Used
    // for the 3D component visualization axis and the gravity component in simulation
    // (F = m * g * sin(angle) along the rack).
    public static final double RACK_ANGLE_RAD = Units.degreesToRadians(23.0);
    // From CAD mass properties: 10.863 lb carriage assembly (includes the NEO; excludes screws
    // and nuts, so the real value runs slightly heavier)
    public static final double CARRIAGE_MASS_KG = 4.93;

    // RIO-side profiled PID gains and constraints — PLACEHOLDER starting values (sim-workable;
    // retune on the real mechanism before first competition use)
    public static final double LINEAR_P = 40.0; // PLACEHOLDER
    public static final double LINEAR_I = 0.0;
    public static final double LINEAR_D = 0.0;
    public static final double LINEAR_MAX_VELOCITY_METERS_PER_SEC = 0.5; // PLACEHOLDER
    public static final double LINEAR_MAX_ACCELERATION_METERS_PER_SEC_SQ = 2.0; // PLACEHOLDER
    public static final double LINEAR_POSITION_TOLERANCE_METERS = 0.01; // PLACEHOLDER

    // Fast force-retract ("come home now") — a deliberately aggressive profile for the emergency
    // stow button, faster than the normal positioning constraints above. PLACEHOLDER.
    public static final double FAST_RETRACT_MAX_VELOCITY_METERS_PER_SEC = 1.0; // PLACEHOLDER
    public static final double FAST_RETRACT_MAX_ACCELERATION_METERS_PER_SEC_SQ = 4.0; // PLACEHOLDER

    // NEO protection: REVLib smart current limits (amps). The holding limit is applied when
    // maintaining position against the retracted hardstop; the homing limit is applied while
    // intentionally stalling into the hardstop. PLACEHOLDER starting values.
    public static final int LINEAR_NORMAL_CURRENT_LIMIT_AMPS = 30;
    public static final int LINEAR_HOMING_CURRENT_LIMIT_AMPS = 10;
    public static final int LINEAR_HOLDING_CURRENT_LIMIT_AMPS = 5;

    // Homing: hold the home button to drive slowly into the retracted hardstop and detect the
    // stall via current rise + velocity stagnation. PLACEHOLDER values — verify direction (sign)
    // and thresholds on the real mechanism before first power-on.
    // Negative = retract (motor is inverted, see LINEAR_MOTOR_INVERTED). Slowed from -1.5 V.
    public static final double HOMING_VOLTS = -1.5;
    // A NEO stalled at 1 V only draws about 8.8 A, so the threshold sits well below that
    public static final double HOMING_STALL_CURRENT_AMPS = 8.0;
    public static final double HOMING_STALL_VELOCITY_METERS_PER_SEC = 0.005;
    public static final double HOMING_STALL_DEBOUNCE_SECS = 0.25;
    // Longer than before because the slower speed can take several seconds over full travel
    public static final double HOMING_TIMEOUT_SECS = 10.0;

    // Thermal monitoring — PLACEHOLDER warning threshold
    public static final double MOTOR_TEMP_WARNING_CELSIUS = 80.0;

    // Firing aggregation ("shutter"): while shooting, the intake sweeps fully in to herd balls,
    // then oscillates between the retracted position and this partially-out position until firing
    // stops. PLACEHOLDER stroke length — tune for effective ball agitation.
    public static final double SHUTTER_OUT_POSITION_METERS = 0.10;

    // Aggregation motion speed — deliberately slower than normal positioning so the intake herds
    // balls instead of batting them away. PLACEHOLDER — tune against real ball behavior.
    public static final double AGGREGATE_MAX_VELOCITY_METERS_PER_SEC = 0.15;
    public static final double AGGREGATE_MAX_ACCELERATION_METERS_PER_SEC_SQ = 0.8;

    // Rollers — PLACEHOLDER starting values: tune the intake/eject speeds on the real mechanism
    public static final double ROLLER_INTAKE_VOLTS = 8.0;
    public static final double ROLLER_EJECT_VOLTS = -6.0;
    public static final int ROLLER_CURRENT_LIMIT_AMPS = 40;
  }

  /**
   * Shooter: drum flywheel (2x Falcon 500, master + inverted hardware follower, onboard Phoenix 6
   * velocity control) and kicker (1x NEO, 3:1 gearbox). All CAN IDs, gains, setpoints, and the
   * distance-to-RPM map are PLACEHOLDERS to be populated during tuning.
   */
  public static final class Shooter {
    // CAN IDs — PLACEHOLDER assignments, but note the legal FRC CAN device ID range is 0-62;
    // out-of-range IDs are silently non-functional (drive uses 1-8, CANcoders 26-29, Pigeon 54)
    public static final int DRUM_MASTER_ID = 52;
    public static final int DRUM_FOLLOWER_ID = 22;
    public static final int KICKER_MOTOR_ID = 31;

    // Drum belt path: 12t pulley (Falcon) -> 105t pulley (intermediate) -> 15t pulley (drum).
    // With one pulley per shaft the intermediate 105t cancels out of the end-to-end ratio, so the
    // net reduction is 15/12 = 1.25 motor rotations per drum rotation (drum spins at 0.8x motor).
    // ASSUMPTION: if the 105t shaft actually carries a second (unstated) pulley, this ratio is
    // wrong — reverify against CAD before trusting shot speeds.
    public static final double DRUM_GEAR_RATIO = 15.0 / 12.0;

    // Drum physical model, from CAD mass properties (drum + wheels assembly: 3.6301 kg,
    // Lxx = 15.966 lb-in^2 about the spin axis). Radius is 2 in — the drum runs 4 in wheels.
    public static final double DRUM_MOI_KG_M2 = 0.00467;
    public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(2.0);

    // Drum onboard (Phoenix 6 device-level) velocity PID gains. kV is derived from the Falcon 500
    // free speed (6380 RPM = 106.3 rot/s at 12 V -> ~0.113 V per motor rot/s); kP is a gentle
    // starting value. PLACEHOLDER — retune on the real mechanism.
    // Tuned 2026-09-22 at 2000 RPM (no balls): spin-up peaks ~2050 at 0.3, ~2060 at 0.4, 0.6+
    // oscillates. If spin-up overshoot matters later, Motion Magic Velocity (ramped setpoint + kA)
    // would allow a stronger kP.
    public static final double DRUM_P = 0.3;
    public static final double DRUM_I = 0.0;
    public static final double DRUM_D = 0.0;
    public static final double DRUM_S =
        0.25; // Static friction FF (volts) — measured: starts at 0.25 V
    // Velocity FF (volts per motor rot/s) — measured 2026-09-22 with kP=0: 0.108 @1500, 0.109
    // @2000, 0.110 @2500 drum RPM (was the 12 / 106.3 = 0.113 spec-sheet value)
    public static final double DRUM_V = 0.109;

    // Drum motor protection — PLACEHOLDER starting values
    public static final double DRUM_STATOR_CURRENT_LIMIT_AMPS = 60.0;
    public static final double DRUM_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

    // Vision distance (meters) -> drum RPM interpolation map. Starting values from the trajectory
    // calculator (64 degree hood, eta=0.9, exit height 17.973 in) — verify with real shot tuning.
    // NOTE: no ballistic solution exists below ~1.7 m at this hood angle; the interpolation map
    // clamps to the 1.7 m entry for closer shots. Rows are {distanceMeters, rpm}; distances MUST
    // be strictly increasing and distinct (validated at Shooter construction).
    public static final double[][] DISTANCE_TO_RPM_MAP = {
      {1.7, 1666.0},
      {2.2, 1714.0},
      {2.7, 1826.0},
      {3.2, 1947.0},
      {3.7, 2068.0},
      {4.3, 2211.0},
    };

    // isReadyToShoot(): drum velocity must stay within this tolerance of the vision-mapped target
    // for the debounce period — PLACEHOLDER (tighten during tuning; "razor-thin" per design)
    public static final double DRUM_READY_TOLERANCE_RPM = 50.0;
    public static final double DRUM_READY_DEBOUNCE_SECS = 0.1;
    // Hysteresis: once ready, feeding continues until the drum sags MORE than this below target.
    // Each ball pulls the drum down on contact; without this the feed stutters on every ball.
    // PLACEHOLDER — set just larger than the per-ball dip seen in Shooter/Drum/VelocityErrorRpm.
    // Measured 2026-09-22 at 2000 RPM, kP 0.3: a full 3-ball volley dips ~500 RPM in 0.02 s and
    // recovers in ~0.085 s, so 600 leaves margin without letting a genuinely slow drum feed.
    public static final double DRUM_READY_DROP_ALLOWANCE_RPM = 600.0;

    // isReadyToShoot() also requires the robot heading to be within this tolerance of the hub
    // bearing (the hood is fixed, so aim IS heading). The hub radius (0.5969 m) subtends ~8 deg at
    // the longest table distance (4.3 m) — 5 deg leaves margin. PLACEHOLDER — tune on the field.
    public static final double AIM_TOLERANCE_DEG = 5.0;

    // Vision-loss fallback: vision cannot be relied on, so losing it must never disable the
    // shooter. With no recent vision correction the pose-derived distance is untrustworthy, so the
    // drum falls back to this fixed setpoint and the driver ranges by eye. PLACEHOLDER — pick the
    // RPM for the distance you most commonly shoot from.
    public static final double VISION_FALLBACK_RPM = 1900.0;

    // Neutral-zone funneling: fixed lob RPM toward the alliance corner (distance-to-corner varies
    // and precision doesn't matter — just get fuel back to friendly territory). PLACEHOLDER.
    public static final double FUNNEL_RPM = 2300.0;

    // Idle spin: the drum is kept spinning at this low RPM whenever the robot is enabled and not
    // actively shooting, so spin-up to a shot RPM is a small step instead of from a dead stop.
    // Low enough that it can never satisfy the ready-to-shoot interlock (which requires an active
    // spin-up request anyway). PLACEHOLDER — keep just high enough to help spin-up.
    public static final double SHOOTER_IDLE_RPM = 0.0;

    // Kicker (NEO through a 3:1 gearbox, basic voltage control) — PLACEHOLDER speeds
    public static final double KICKER_GEAR_RATIO = 3.0;
    // Feed volts drive BPS: the harder the kicker shoves balls into the drum, the faster the
    // volley. Raised 6 -> 10 to push throughput; can go toward 12 if the drum keeps balls on
    // target. Watch Shooter/Drum/VelocityErrorRpm — if the per-ball sag grows past the drum's
    // recovery, shots scatter (raise DRUM_STATOR_CURRENT_LIMIT_AMPS or back this down).
    public static final double KICKER_FEED_VOLTS = 10.0; // Tune on real mechanism
    public static final double KICKER_REVERSE_VOLTS =
        -4.0; // Moderate reverse for unjam; PLACEHOLDER
    // Raised 30 -> 40 so the NEO doesn't current-clip at the higher feed voltage
    public static final int KICKER_CURRENT_LIMIT_AMPS = 40;

    // Chute unjam: spin the drum FORWARD a little faster than idle to fling a ball stuck in the
    // chute clear (while the kicker/indexer back the jam out). PLACEHOLDER — a touch above idle.
    public static final double UNJAM_DRUM_RPM = 1200.0;

    // Thermal monitoring — PLACEHOLDER warning threshold
    public static final double MOTOR_TEMP_WARNING_CELSIUS = 80.0;

    // Firing-sequence orchestration (used by RobotContainer's aim-and-shoot):
    // How long R2 is held (from press) before the intake stops collecting and sweeps in to herd
    // the remaining balls. PLACEHOLDER — tune to how long a volley takes to clear staged balls.
    public static final double SHOOT_HERD_DELAY_SECS = 2.0;
    // While aiming+shooting, cap translation to this fraction of max speed so shoot-on-the-move
    // lead error stays inside the sim-validated envelope. PLACEHOLDER.
    public static final double SHOOT_ON_MOVE_SPEED_SCALAR = 0.5;

    // --- Physical shot geometry (shared by the live trajectory prediction, which runs on the
    // real robot too, and by the simulation's launch model) ---
    // Fixed hood angle
    public static final double HOOD_ANGLE_DEG = 64.0;
    // Ball exit point from CAD: height above the floor, and XY offset from robot center
    public static final double BALL_EXIT_HEIGHT_METERS = Units.inchesToMeters(19.179766);
    public static final Translation2d BALL_EXIT_OFFSET =
        new Translation2d(Units.inchesToMeters(8.246225), 0.0);
    // Drum-surface-speed -> ball-exit-speed transfer ratio, back-solved from the RPM lookup table
    // so the table's RPMs physically drop into the hub (fit 0.63-0.67 across 1.7-4.3 m; vacuum
    // ballistics). NOT the trajectory calculator's eta.
    public static final double SURFACE_TO_BALL_SPEED_RATIO = 0.66;
  }

  /**
   * Indexer belt floor (1x Kraken X60 via Phoenix 6). CAN ID and speeds are PLACEHOLDERS; the
   * stator limit is a conservative jam-protection starting value.
   */
  public static final class Indexer {
    // CAN ID — PLACEHOLDER assignment, but note the legal FRC CAN device ID range is 0-62
    public static final int INDEXER_MOTOR_ID = 20;

    // Belt gearing: 16t (Kraken) -> 70t (belt drive shaft), so 70/16 = 4.375 motor rotations per
    // belt-drive rotation. Derived from the mechanism, not tuned.
    public static final double INDEXER_GEAR_RATIO = 70.0 / 16.0;

    // Motor protection: strict stator limit so a jammed game piece cannot burn out the motor —
    // PLACEHOLDER starting values
    public static final double INDEXER_STATOR_CURRENT_LIMIT_AMPS = 40.0;
    public static final double INDEXER_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

    // Belt speeds — tune on the real mechanism. Raised 6 -> 10 to keep the belt supplying balls to
    // the kicker at least as fast as the kicker fires them (a slow belt would starve the kicker and
    // cap BPS regardless of kicker speed). Keep at or above the kicker's effective feed rate.
    public static final double INDEXER_FEED_VOLTS = 10.0;
    // Reverse (unjam) — deliberately gentler than the kicker's reverse (-4.0) so the belt eases the
    // jam back rather than yanking it. PLACEHOLDER.
    public static final double INDEXER_REVERSE_VOLTS = -2.5;

    // Thermal monitoring — PLACEHOLDER warning threshold
    public static final double MOTOR_TEMP_WARNING_CELSIUS = 80.0;
  }
}
