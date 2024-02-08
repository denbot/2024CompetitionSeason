// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystens.Intake; TODO: coordinate with intake to import the right subsystem

public class Shoot extends Command {
  private final Shooter shooter;
  private double speed = 1; // Change to intake motor speed (in rotations per second)
  private Timer timer = new Timer();

  public Shoot(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (timer.hasElapsed(2)){
      shooter.setDefault();
    } else {
      if (shooter.motorsAtShootingSpeed) { // TODO: Add "and intake has a peice" 
        shooter.shoot();
        // TODO: Interact with intake subsystem to run belt
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
