package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionConstants;
import org.junit.jupiter.api.Test;

/** Checks the neutral zone and funnel targets against the RoboCon field geometry. */
class FieldConstantsTest {
  private static final double FIELD_LENGTH = VisionConstants.aprilTagLayout.getFieldLength();
  private static final double FIELD_WIDTH = VisionConstants.aprilTagLayout.getFieldWidth();

  @Test
  void allianceZonesAreNotNeutral() {
    // Typical shooting spots ~2.5 m in front of each hub, inside each alliance zone
    assertTrue(FieldConstants.getFunnelTarget(new Translation2d(1.5, 4.0)).isEmpty());
    assertTrue(
        FieldConstants.getFunnelTarget(new Translation2d(FIELD_LENGTH - 1.5, 4.0)).isEmpty());
  }

  @Test
  void midfieldIsNeutral() {
    assertTrue(
        FieldConstants.getFunnelTarget(new Translation2d(FIELD_LENGTH / 2, 2.0)).isPresent());
    assertTrue(
        FieldConstants.getFunnelTarget(new Translation2d(FIELD_LENGTH / 2, 6.0)).isPresent());
  }

  @Test
  void funnelTargetsAreOnTheField() {
    for (Translation2d target :
        new Translation2d[] {
          FieldConstants.BLUE_FUNNEL_TARGET_LOW_Y,
          FieldConstants.BLUE_FUNNEL_TARGET_HIGH_Y,
          FieldConstants.RED_FUNNEL_TARGET_LOW_Y,
          FieldConstants.RED_FUNNEL_TARGET_HIGH_Y
        }) {
      assertTrue(target.getX() > 0 && target.getX() < FIELD_LENGTH, "X on field: " + target);
      assertTrue(target.getY() > 0 && target.getY() < FIELD_WIDTH, "Y on field: " + target);
    }
  }
}
