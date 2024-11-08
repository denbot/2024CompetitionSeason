// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoCenter extends Command {
  
  boolean end = false;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  
  /** Creates a new AutoCenter. */
  public AutoCenter() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
    if (LimelightHelpers.getTX("") == 0) {
        end = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTX("") > 2.5) {
      drive.withRotationalRate(-0.1);
    } else if (LimelightHelpers.getTX("") < 2.5) {
      drive.withRotationalRate(0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
