// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandHolder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoIntakeCommand extends Command {
  private final CommandHolder commands;
    private final Intake intake;
    private final Shooter shooter;

    private final Timer timer = new Timer();

    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final NeutralOut brake = new NeutralOut();
  
    boolean noteAtShooterSensor;
    boolean noteTrippedShooterSensor = false;
    boolean notePassedShooterSensor = false;
    /** Creates a new IntakeCommand. */
  public AutoIntakeCommand(CommandHolder commands,
  Intake intake,
  Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.commands = commands;
    this.intake = intake;
    this.shooter = shooter;
    addRequirements(intake);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Hello");
    intake.setMotorControl(voltageOut.withOutput(4));
    shooter.readyArmForNewNote();
    timer.start();
    noteAtShooterSensor = false;
    noteTrippedShooterSensor = false;
    notePassedShooterSensor = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Timer", timer.get());
    noteAtShooterSensor  = intake.noteAtShooterSensor();

    if (noteAtShooterSensor) {
      noteTrippedShooterSensor = true;
    }

    if (noteTrippedShooterSensor &! noteAtShooterSensor &! notePassedShooterSensor) {
      notePassedShooterSensor = true;
      timer.restart();
      intake.setMotorControl(voltageOut.withOutput(-1.2));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setMotorControl(brake);
    shooter.setNoteInShooter(true);
    System.out.println("goodbye");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.hasElapsed(0.17) & notePassedShooterSensor) || timer.hasElapsed(3);
  }
}
