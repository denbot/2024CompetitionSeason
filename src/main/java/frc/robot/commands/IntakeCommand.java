// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {

  private final Intake intake;
  private double speed = 0;
  private boolean intakedNote = false;

  /** Creates a new PrepCommand. */
  public IntakeCommand(Intake intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.intake.noteInIntake() && !intakedNote) {
      intake.setIntakeSpeed(2.4); // TODO: tune
      intakedNote = true;
      return;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      intakedNote = false;
      intake.setIntakeSpeed(0);
      return;
    }
    Timer timer = new Timer();

    timer.start();
    intake.setIntakeSpeed(-1.2);

    while (!timer.hasElapsed(0.05));

    timer.stop();
    timer.reset();

    intakedNote = false;
    intake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakedNote && !intake.noteInIntake();
  }
}
