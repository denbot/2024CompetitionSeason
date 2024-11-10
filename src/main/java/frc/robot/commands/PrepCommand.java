// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter;

/**
 * Prepare the shooter for a specific angle and speed
 */
public class PrepCommand extends Command {
    public static PrepCommand currentPrepCommand = null;

    private final Shooter shooter;
    private double angle;
    private double speed;

    private boolean autoAim = false;

    public PrepCommand(Shooter shooter, double angle, double speed, boolean autoAim) {
        addRequirements(shooter);
        this.shooter = shooter;
        this.angle = angle;
        this.speed = speed;
        this.autoAim = autoAim;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentPrepCommand = this;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        // if we want to auto aim, take our distance from the april tag and convert it to inches
        // then plug it into the regression and change the angle to the (clamped) estimated angle 
        if (autoAim) {
            double xDistance = LimelightHelpers.getTargetPose_RobotSpace("")[0];
            double yDistance = LimelightHelpers.getTargetPose_RobotSpace("")[2];
            
            xDistance = Units.metersToInches(xDistance);
            yDistance = Units.metersToInches(yDistance);
            
            double distance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
            
            double estimatedAngle = 215.172 * Math.pow(distance, -0.3416);
            
            SmartDashboard.putNumber("estimated angle", estimatedAngle);
            angle = Math.max(35, Math.min(75, estimatedAngle));
        }
        
        shooter.setAngle(angle);
        shooter.startMotors(speed);
        shooter.setNoteReadyToFire(true);

        if(currentPrepCommand == this) {
            SmartDashboard.putNumber("Prep command Angle", angle);
            SmartDashboard.putNumber("Prep command Speed", speed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // initialize of the other command should happen after the end of this one, but just in case we compare instances
        if(currentPrepCommand == this) {
            currentPrepCommand = null;
        }
    }

    public void changeAngle(double angleDelta) {
        angle += angleDelta;
    }

    public void changeSpeed(double speedDelta) {
        this.speed += speedDelta;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
