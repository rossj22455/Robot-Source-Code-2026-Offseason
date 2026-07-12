// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

/**
 * Constants for the indexer belt floor (1x Kraken X60 via Phoenix 6). CAN ID and speeds are
 * PLACEHOLDERS; the stator limit is a conservative jam-protection starting value.
 */
public class IndexerConstants {
  // CAN ID — PLACEHOLDER assignment, but note the legal FRC CAN device ID range is 0-62
  public static final int INDEXER_MOTOR_ID = 43;

  // Belt gearing: 16t (Kraken) -> 70t (belt drive shaft), so 70/16 = 4.375 motor rotations per
  // belt-drive rotation. Derived from the mechanism, not tuned.
  public static final double INDEXER_GEAR_RATIO = 70.0 / 16.0;

  // Motor protection: strict stator limit so a jammed game piece cannot burn out the motor —
  // PLACEHOLDER starting values
  public static final double INDEXER_STATOR_CURRENT_LIMIT_AMPS = 40.0;
  public static final double INDEXER_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

  // Belt speeds — PLACEHOLDER starting values: tune on the real mechanism
  public static final double INDEXER_FEED_VOLTS = 6.0;
  public static final double INDEXER_REVERSE_VOLTS = -4.0;

  // Thermal monitoring — PLACEHOLDER warning threshold
  public static final double MOTOR_TEMP_WARNING_CELSIUS = 80.0;
}
