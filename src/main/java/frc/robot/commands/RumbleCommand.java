// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;

public class RumbleCommand extends Command {
    /**
     * Creates a new Rumble.
     */

    private GenericHID controller;
    private double power;
    private double time;
    private Timer timer;

    public RumbleCommand(GenericHID controller, double power, double time) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.controller = controller;
        this.time = time;
        this.power = power;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        controller.setRumble(RumbleType.kBothRumble, power);
        timer.reset();
        timer.start();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        controller.setRumble(RumbleType.kBothRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }
}
