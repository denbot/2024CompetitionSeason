package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.FieldUtil;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.generated.VisionConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configPathPlanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void optimizeCan() {
        for (int i = 0; i < Modules.length; i++) {
            SwerveModule module = Modules[i];
            TalonFX.optimizeBusUtilizationForAll(module.getDriveMotor(), module.getSteerMotor());
        }
    }

    @Override
    public void periodic() {
        m_odometry.update(getGyroYaw(), m_modulePositions);
        Pose2d pose = m_odometry.getEstimatedPosition();
        SmartDashboard.putNumberArray("pose estimation",
                new Double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });
    }

    public void zeroGyro() {
        m_pigeon2.setYaw(0);
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void configPathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> getState().Pose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeedsAuto, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                Constants.AutoConstants.autoConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void resetPose(Pose2d pose) {
        this.seedFieldRelative(pose);
        this.zeroGyroAdjusted(pose.getRotation());
    }

    public void setGyroYaw(Rotation2d yaw) {
        m_pigeon2.setYaw(yaw.getDegrees());
    }

    /**
     * Will rotate the provided value by 180 if on red alliance
     */
    public void zeroGyroAdjusted(Rotation2d rot) {
        setGyroYaw(FieldUtil.isAllianceBlue() ? rot : rot.plus(Rotation2d.fromDegrees(180)));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * Hijacks the robot's rotation but keeps PathPlanner translation. Used for
     * shooting while following the path
     *
     * @param speeds
     */
    public void setChassisSpeedsAuto(ChassisSpeeds speeds) {
        setControl(autoRequest.withSpeeds(speeds));
    }

    public void setAngleoffLimelight() {

    }

    public Rotation2d getGyroYaw() {
        double rawYaw = m_pigeon2.getYaw().getValue();
        double yawWithRollover = rawYaw > 0 ? rawYaw % 360 : 360 - Math.abs(rawYaw % 360);

        return Rotation2d.fromDegrees(yawWithRollover);
    }

    public void updateVision() {
        if (LimelightHelpers.getBotPose2d_wpiBlue("").getX() < 0.1) {
            return;
        }

        double tagDistance = LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.limelightName)
                .getTranslation().getNorm(); // Find direct distance to target for std dev calculation
        if (tagDistance < VisionConstants.maxUsableDistance) {

            double xyStdDev2 = VisionConstants.calcStdDev(tagDistance);

            Pose2d poseFromVision = LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.limelightName);
            double poseFromVisionTimestamp = Timer.getFPGATimestamp()
                    - (LimelightHelpers.getLatency_Capture(VisionConstants.limelightName)
                            + LimelightHelpers.getLatency_Pipeline(VisionConstants.limelightName)) / 1000;

            Pose2d withGyroData = new Pose2d(poseFromVision.getTranslation(), getGyroYaw());

            addVisionMeasurement(withGyroData, poseFromVisionTimestamp,
                    VecBuilder.fill(xyStdDev2, xyStdDev2, 0));
        }
    }
}
