import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestInstance.Lifecycle;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.lib.util.FieldUtil;
import frc.robot.commands.AutoPrepCommand;
import frc.robot.generated.SwerveTunerConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

@TestInstance(Lifecycle.PER_CLASS)
class AutoPrepTest {
    static final double ERROR = 10; // we just don't want something outragous
    private final SwerveSubsystem drivetrain = SwerveTunerConstants.DriveTrain; // My drivetrain
    private Shooter shooter = new Shooter();
    private AutoPrepCommand autoPrepCommand;

    @BeforeAll
    void init() {
        shooter.shooterInit();
        autoPrepCommand = new AutoPrepCommand(shooter, drivetrain, 120);
    }

    @Test
    void blueCenterSubwooferShoot() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        Translation3d speakerCenter = FieldUtil.simGetCenterOfAllianceSpeakerOpening();

        drivetrain.resetPose(new Pose2d(
            new Translation2d(1.36, speakerCenter.getY()),
            new Rotation2d(0)));

        double angle = autoPrepCommand.calculateArmAngleFromRotationCenter(speakerCenter, drivetrain.getPose().getTranslation());
        assertEquals(65, angle, ERROR);
    }

    @Test
    void redCenterSubwooferShoot() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        Translation3d speakerCenter = FieldUtil.simGetCenterOfAllianceSpeakerOpening();

        drivetrain.resetPose(new Pose2d(
            new Translation2d(15.2, 5.5),
            new Rotation2d(0)));

        double angle = autoPrepCommand.calculateArmAngleFromRotationCenter(speakerCenter, drivetrain.getPose().getTranslation());
        assertEquals(65, angle, ERROR);
    }
}
