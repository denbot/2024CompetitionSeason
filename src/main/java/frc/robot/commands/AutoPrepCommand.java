package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoPrepCommand extends Command {
  Shooter shooter;
  SwerveSubsystem swerve;
  double speed;

  public AutoPrepCommand(Shooter shooter, SwerveSubsystem swerve, double speed) {
    this.shooter = shooter;
    this.swerve = swerve;
    this.speed = speed;
    addRequirements(shooter, swerve);
  }

  @Override
  public void execute() {
    swerve.updateVision();
    Translation3d target = FieldUtil.getCenterOfAllianceSpeakerOpening();

    double diffHeight = target.getZ() - Constants.MechanicalConstants.armBaseOffsetZ;
    double targetDistance = target.toTranslation2d().getDistance(swerve.getPose().getTranslation());

    // TODO: uncomment and tune when pid for swerve is completed
    // Supplier<Rotation2d> robotAngle = () -> Rotation2d.fromRadians( // Find the angle to turn the robot to
    //     Math.atan((PoseEstimation.getEstimatedPose().getY() - target.getY())
    //         / (PoseEstimation.getEstimatedPose().getX() - target.getX())));

    // should do shooter regression instead when we have the time
    double targetAngle = Math.toDegrees(Math.atan(diffHeight / targetDistance));

    shooter.setAngle(targetAngle);
    shooter.startMotors(speed);
  }

  @Override
  public boolean isFinished() {
    return shooter.canShoot();
  }
}
