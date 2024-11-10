// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoCenter extends Command {
  
  boolean end = false;
  double kP = -0.05;
  private final Timer timer = new Timer();
  private final SwerveSubsystem driveSubsystem;
  private final SwerveRequest.FieldCentric drive;

  private final ShootCommand shootCommand = new ShootCommand(RobotContainer.shooterSubsystem, RobotContainer.intakeSubsystem);

  private final PrepCommand prep = new PrepCommand(RobotContainer.shooterSubsystem, 45, 50, true);
  
  /** Creates a new AutoCenter. */
  public AutoCenter(SwerveSubsystem driveSubsystem, SwerveRequest.FieldCentric drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    end = false;
    if (LimelightHelpers.getTX("") == 0) {
        end = true;
    }

    prep.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setControl(drive.withRotationalRate(LimelightHelpers.getTX("")*kP));
    if (Math.abs(LimelightHelpers.getTX("")) < 5) {
      timer.start();
    } else {
      timer.reset();
    }

    // if (LimelightHelpers.getTX("") > 2.5) {
    //   System.out.println("trying to drive clockwise");
    //   driveSubsystem.setControl(drive.withRotationalRate(-0.8));
    // } else if (LimelightHelpers.getTX("") < -2.5) {
    //   System.out.println("trying to drive counterclockwise");
    //   driveSubsystem.setControl(drive.withRotationalRate(0.8));
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (end || timer.get() > 0.5);
  }
}
