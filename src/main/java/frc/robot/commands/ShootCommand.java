// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ShootCommand extends Command {
  private final Shooter shooter;
  private final Intake intake;
  private double speed = 0.2; // Intake motor speed (in percentage of power)
  private Timer timer = new Timer();

  public ShootCommand(Shooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(shooter);
    addRequirements(intake);
  }

  public void initialize() {
    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() == 0 && shooter.canShoot()) {
      intake.shoot(speed);
    } 
    timer.start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
    shooter.readyArmForNewNote();
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
  }
}
