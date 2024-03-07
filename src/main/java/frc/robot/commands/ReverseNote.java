// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ReverseNote extends Command {
  /** Creates a new Eject. */
  Intake intake;
  Shooter shooter;
  private double reverseSpeed;
  public ReverseNote(Intake intake, Shooter shooter, double ejectSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.reverseSpeed = ejectSpeed;
    this.intake = intake;
    this.shooter = shooter;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.noteInIntake()) {
      intake.eject(reverseSpeed, false);
    } else {
      intake.eject(reverseSpeed, true);
      shooter.intake(reverseSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.noteInIntake();
  }
}
