package frc.lib.util;

import java.util.Optional;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class FieldUtil {
    // dimensions: https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf

    // page 239
    public static final Translation2d BlueSpeakerPosition = new Translation2d(8.27 - 8.308467, 4.105 + 1.442593);
    public static final Translation2d RedSpeakerPosition = new Translation2d(8.27 + 8.308467, 4.105 + 1.442593);

    // page 120
    public static final double SpeakerHeight = Units.inchesToMeters(78.13);
    public static final double SpeakerCoverHeight = Units.inchesToMeters(82.90);

    public static final double SpeakerOpeningHeight = (SpeakerCoverHeight - SpeakerHeight) / 2 + SpeakerHeight;
    public static final double SpeakerOpeningLength = Units.inchesToMeters(18.11);
    public static final double SpeakerOpeningWidth = Units.inchesToMeters(41.38);


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

    public static Translation2d simGetAllianceSpeakerPosition() {
        return DriverStationSim.getAllianceStationId() == AllianceStationID.Blue1 ? BlueSpeakerPosition : RedSpeakerPosition;
    }

    public static Translation3d getCenterOfAllianceSpeakerOpening() {
        if (isAllianceBlue()) {
            return new Translation3d(
                    getAllianceSpeakerPosition().getX() + SpeakerOpeningLength / 2.0,
                    getAllianceSpeakerPosition().getY(),
                    SpeakerOpeningHeight);
        }

        return new Translation3d(
                getAllianceSpeakerPosition().getX() - SpeakerOpeningLength / 2.0,
                getAllianceSpeakerPosition().getY(),
                SpeakerOpeningHeight);
    }

    public static Translation3d simGetCenterOfAllianceSpeakerOpening() {
        if (DriverStationSim.getAllianceStationId() == AllianceStationID.Blue1) {
            return new Translation3d(
                    simGetAllianceSpeakerPosition().getX() + SpeakerOpeningLength / 2.0,
                    simGetAllianceSpeakerPosition().getY(),
                    SpeakerOpeningHeight);
        }

        return new Translation3d(
                simGetAllianceSpeakerPosition().getX() - SpeakerOpeningLength / 2.0,
                simGetAllianceSpeakerPosition().getY(),
                SpeakerOpeningHeight);

    }
}
