package frc.robot.generated;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public static double calcStdDev(double metersFromTarget) {

    // double inches = Units.metersToInches(metersFromTarget); // Convert to inches
    // double hypotenuse = Math.sqrt(Math.pow(inches, 2) + Math.pow(57, 2)); //
    // Account for april tag being high off the ground
    // double calculated = StdDevScalar * Math.pow(Math.pow(hypotenuse, 19.5335),
    // -0.1/2); // Plug into Std Dev equation that we got experimentally

    // return MathUtil.clamp(0.01 * Math.pow(metersFromTarget, 3), 0, 1);
    return 0.165 * Math.pow(metersFromTarget, 2);
  }

  public static final double maxUsableDistance = 5.25; // Meters

  public static final String limelightName = "";

  public static final Translation3d limelightOffset = new Translation3d(
      Units.inchesToMeters(-11.384098),
      Units.inchesToMeters(-8.607840), Units.inchesToMeters(14.617002));

  // Limelight rotation: Pitch:10
}
