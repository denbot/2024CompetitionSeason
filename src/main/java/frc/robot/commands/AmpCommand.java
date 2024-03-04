// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AmpCommand extends Command {
  
  private final Shooter shooter;
  private double speed = 1; // TODO: Change to amp motor speed (in rotations per second)
  private final double ampAlignAngle = 36; // TODO: tune relative to shooting from right in front of amp

  public AmpCommand(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setAngle(ampAlignAngle);
    shooter.startMotors(speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
