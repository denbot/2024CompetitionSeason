package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
import frc.robot.Constants;
import frc.robot.Constants.MechanicalConstants;
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

        // TODO: have a way to rotate the robot before using this value
        Rotation2d robotAngle = new Rotation2d(
                Math.atan((swerve.getPose().getY() - target.getY()) / (swerve.getPose().getX() - target.getX()))
        );

        // Supplier<Rotation2d> robotAngle = () -> Rotation2d.fromRadians( // Find the
        // angle to turn the robot to
        // Math.atan((PoseEstimation.getEstimatedPose().getY() - target.getY())
        // / (PoseEstimation.getEstimatedPose().getX() - target.getX())));

        // maybe should do shooter regression instead if we have time
        double targetAngle = calculateArmAngleFromRotationCenter(target, swerve.getPose().getTranslation());

        shooter.setAngle(targetAngle);
        shooter.startMotors(speed);
    }

    /***
     * finds angle that the wrist should rotate so that the shoooter goes thorugh
     * the target
     * assumes that the robot is already rotated correctly
     *
     * @param target 3d location of target
     * @param robotPosition x,y position of robot
     * @return angle (degrees)
     */
    public double calculateArmAngleFromRotationCenter(Translation3d target, Translation2d robotPosition) throws Error {
        /*
         * the following is a lot of math
         * we first find the length of a tangent line from the target
         * with that length as a radius of a circle, we find the intersection points
         * with a circle with r=length of perpendicular line from center of rotation to
         * arm base
         * with the intersection point, we find the angle
         *
         *
         * formula for intersection:
         * https://paulbourke.net/geometry/circlesphere
         */
        // 2d target triangle x/y vals
        Translation2d targetTriangle = new Translation2d(
                Math.sqrt(Math.pow(target.getX() - robotPosition.getX(), 2) + Math.pow(target.getY() - robotPosition.getY(), 2)),
                target.getZ() - Constants.MechanicalConstants.armBaseOffsetZ);

        // diagnol distance from target to robot
        double dist = Math.sqrt(
                Math.pow(Math.abs(targetTriangle.getX()) - Constants.MechanicalConstants.armBaseOffsetX, 2)
                + Math.pow(targetTriangle.getY(), 2));

        // length of tangent line from target
        double r = Math.sqrt(Math.pow(dist, 2) - Math.pow(MechanicalConstants.armRotationRadius, 2));

        double a = (Math.pow(Constants.MechanicalConstants.armRotationRadius, 2) - Math.pow(r, 2) + Math.pow(dist, 2))
                / (2 * dist);
        double h = Math.sqrt(Math.pow(Constants.MechanicalConstants.armRotationRadius, 2) - Math.pow(a, 2));

        double x2 = a * targetTriangle.getX() / dist;
        double y2 = a * targetTriangle.getY() / dist;
        double intersectionX = x2 + h * (targetTriangle.getY()) / dist;
        double intersectionY = y2 - h * (targetTriangle.getX()) / dist;

        double angle = Math.toDegrees(Math.atan(intersectionY / intersectionX)) + 90;
        if (angle > MechanicalConstants.maxArmAngle || angle < MechanicalConstants.minArmAngle) {
            throw new Error("auto prep angle should be in between min/max angles.  trying to set to : " + angle);
        }

        return angle;
    }

    @Override
    public boolean isFinished() {
        return shooter.canShoot();
    }
}
