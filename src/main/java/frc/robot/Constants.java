// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final String canivoreSerial = "D75CCE723353385320202034111303FF";
    }

    public static final class AutoConstants {
        public static final HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(
                // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                6.21, // Max module speed, in m/s
                0.40451045104, // Drive base radius in meters. Distance from robot center to furthest
                // module.
                new ReplanningConfig() // Default path replanning config. See the API for the options
        // here
        );
    }

    public static final class MechanicalConstants {
        // tune constants later: base/pivot of arm from center of robot (in m)
        public static final double armBaseOffsetZ = Units.inchesToMeters(12.864);
        public static final double armBaseOffsetX = Units.inchesToMeters(3.49);
        public static final double armRotationRadius = Units.inchesToMeters(3.0);
    }

    public static final class VisionConstants {
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

        // tune values for robot
        public static final Translation3d limelightOffset = new Translation3d(
                Units.inchesToMeters(-11.384098),
                Units.inchesToMeters(-8.607840), Units.inchesToMeters(14.617002));
    }
}
