package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
import frc.lib.util.LimelightHelpers;
import frc.robot.generated.VisionConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAimArm extends Command {
  Shooter shooter;
  SwerveSubsystem swerve;
  double speed;

  public AutoAimArm(Shooter shooter, SwerveSubsystem swerve, double speed) {
    this.shooter = shooter;
    this.swerve = swerve;
    this.speed = speed;
    addRequirements(shooter, swerve);
  }

  @Override
  public void execute() {
    swerve.updateVision();
    Translation2d target = FieldUtil.getAllianceSpeakerPosition();

    // tune values for speaker / limelight
    double targetHeight = 60;
    double limelightHeight = VisionConstants.limelightOffset.getY(); // might need changing depending on which part of Translation3d is up
    double diffHeight = targetHeight - limelightHeight;

    double targetDistance = target.getDistance(swerve.getPose().getTranslation());


    // figure out this later
    // Supplier<Rotation2d> robotAngle = () -> Rotation2d.fromRadians( // Find the angle to turn the robot to
    //     Math.atan((PoseEstimation.getEstimatedPose().getY() - target.getY())
    //         / (PoseEstimation.getEstimatedPose().getX() - target.getX())));

    // double targetAngle = 40;
    double targetAngle = Math.toDegrees(Math.tan(diffHeight / targetDistance));

    shooter.setAngle(targetAngle);
    shooter.startMotors(speed);
  }

  @Override
  public boolean isFinished() {
    return shooter.canShoot();
  }
}
