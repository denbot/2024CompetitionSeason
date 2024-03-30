// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Prepare the shooter for a specific angle and speed
 */
public class PrepCommand extends Command {
    public static PrepCommand currentPrepCommand = null;

    private final Shooter shooter;
    private double angle;
    private double speed;

    public PrepCommand(Shooter shooter, double angle, double speed) {
        addRequirements(shooter);
        this.shooter = shooter;
        this.angle = angle;
        this.speed = speed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentPrepCommand = this;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
