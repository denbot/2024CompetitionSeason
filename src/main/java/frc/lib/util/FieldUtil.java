package frc.lib.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldUtil {
    public static final Translation2d BlueSpeakerPosition = new Translation2d(8.27 - 8.308467, 4.105 + 1.442593);
    public static final Translation2d RedSpeakerPosition = new Translation2d(8.27 + 8.308467, 4.105 + 1.442593);

    private static final double INCH_TO_CM_RATIO = 2.54;
    private static final double CM_TO_M_RATIO = 0.01;

    public static final double SpeakerHeight = INCH_TO_CM_RATIO * 79.0;


    public static boolean isAllianceBlue() {
        boolean isAllianceBlue = true;

        if (DriverStation.getAlliance().isPresent()) {
            isAllianceBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        }

        return isAllianceBlue;
    }
    /**
     * Get the position of whichever alliance you are on
     *
     * @return the speaker position
     */
    public static Translation2d getAllianceSpeakerPosition() {

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue ? BlueSpeakerPosition : RedSpeakerPosition;
        }

        return BlueSpeakerPosition;
    }

    public static Translation3d getCenterOfAllianceSpeakerOpening() {
        double len = INCH_TO_CM_RATIO * 18.0;
        double height = INCH_TO_CM_RATIO * (Math.tan(Math.toRadians(14)) * len + SpeakerHeight);

        if (isAllianceBlue()) {
            return new Translation3d(getAllianceSpeakerPosition().getX() + len / 2.0 * CM_TO_M_RATIO, getAllianceSpeakerPosition().getY(),
                    height / 2.0 * CM_TO_M_RATIO);
        }

        return new Translation3d(getAllianceSpeakerPosition().getX() - len / 2.0 * CM_TO_M_RATIO, getAllianceSpeakerPosition().getY(),
                height / 2.0 * CM_TO_M_RATIO);
    }
}
