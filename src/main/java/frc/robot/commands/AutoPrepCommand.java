package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    // TODO: calculate any offset of the armrotation from robot center
    double targetAngle = Math.toDegrees(Math.atan(diffHeight / targetDistance));

    shooter.setAngle(targetAngle);
    shooter.startMotors(speed);
  }

  /***
   * @param
   * @return angle that the wrist should rotate so that the shoooter goes thorugh the target
   */
  public double calculateArmAngleFromRotationCenter(Translation3d speakerCenter, Pose2d pose) {
    // 2d target triangle x/y vals
    // TODO: account for x offset of arm relative to robot
    Translation2d target = new Translation2d(
        (speakerCenter.getY() - pose.getY()) / pose.getRotation().getCos(),
        speakerCenter.getZ() - Constants.MechanicalConstants.armBaseOffsetZ);
    double dist = Math.sqrt(Math.pow(target.getX(), 2) + Math.pow(target.getY(), 2));
    double r = Math.sqrt(Math.pow(dist, 2) + Math.pow(Constants.MechanicalConstants.armRotationRadius, 2));
    double a = (Math.pow(Constants.MechanicalConstants.armRotationRadius, 2) - Math.pow(r, 2) + Math.pow(dist, 2)) / (2*dist);
    double h = Math.sqrt(Math.pow(Constants.MechanicalConstants.armRotationRadius, 2) - Math.pow(a, 2));
    double x2 = a * target.getX() / dist;
    double y2 = a * target.getY() / dist;
    // intersection points
    double intersectionX = x2+h*(target.getY())/dist;
    double intersectionY = y2-h*(target.getX())/dist;

    double angle = Math.toDegrees(Math.atan(intersectionY / intersectionX));
    return 90 + angle;
  }

  @Override
  public boolean isFinished() {
    return shooter.canShoot();
  }
}
