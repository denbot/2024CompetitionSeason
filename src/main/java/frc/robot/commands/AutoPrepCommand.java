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

        // TODO: uncomment and tune when pid for swerve is completed
        Rotation2d robotAngle = swerve.getGyroYaw();
        // Supplier<Rotation2d> robotAngle = () -> Rotation2d.fromRadians( // Find the
        // angle to turn the robot to
        // Math.atan((PoseEstimation.getEstimatedPose().getY() - target.getY())
        // / (PoseEstimation.getEstimatedPose().getX() - target.getX())));

        // maybe should do shooter regression instead if we have time
        try {
            double targetAngle = calculateArmAngleFromRotationCenter(target, swerve.getPose().getTranslation(), robotAngle);

            shooter.setAngle(targetAngle);
            shooter.startMotors(speed);
        } catch (Error e) {
            this.cancel();
        }

    }

    /***
     * finds angle that the wrist should rotate so that the shoooter goes thorugh
     * the target
     * assumes that the robot is already rotated correctly
     *
     * @param target 3d location of target
     * @param robotPosition x,y position of robot
     * @param heading desired heading robot should be oriented in
     * @return angle (degrees)
     */
    public double calculateArmAngleFromRotationCenter(Translation3d target, Translation2d robotPosition, Rotation2d heading) throws Error {
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
                (target.getY() - robotPosition.getY()) / heading.getCos(),
                target.getZ() - Constants.MechanicalConstants.armBaseOffsetZ);
        // diagnol distance from target to robot
        double dist = Math.sqrt(
                Math.pow(targetTriangle.getX() - Constants.MechanicalConstants.armBaseOffsetX, 2)
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

        double angle = Math.toDegrees(Math.atan(intersectionY / intersectionX));
        if (angle > 0 || angle < 90) {
            throw new Error("auto angle should be within 0 and 90 degrees");
        }

        return angle + 90;
    }

    @Override
    public boolean isFinished() {
        return shooter.canShoot();
    }
}
