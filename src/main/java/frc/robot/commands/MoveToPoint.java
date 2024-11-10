// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToPoint extends Command {
  /** Creates a new MoveToPoint. */

  private final SwerveSubsystem driveSubsystem;
  private final SwerveRequest.RobotCentric drive;
  private final Timer timer = new Timer();

  private final ShootCommand shootCommand = new ShootCommand(RobotContainer.shooterSubsystem, RobotContainer.intakeSubsystem);

  private final PrepCommand prep = new PrepCommand(RobotContainer.shooterSubsystem, 45, 50, false);

  boolean end = false;
  double kP = 1.2;
  double rotationalKP = -0.05;
  double rotationMult = 1;

  int framesDropped = 0;

  public MoveToPoint(SwerveSubsystem driveSubsystem, SwerveRequest.RobotCentric drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.drive = drive;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
    timer.start();
    rotationMult = 1;
    prep.schedule();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] tagPoseRobot = LimelightHelpers.getTargetPose_RobotSpace("");
    Pose3d pose = new Pose3d(new Translation3d(tagPoseRobot[0], tagPoseRobot[1], tagPoseRobot[2]), new Rotation3d(Math.toRadians(tagPoseRobot[3]), Math.toRadians(tagPoseRobot[4]), Math.toRadians(tagPoseRobot[5])));

    if (LimelightHelpers.getTV("")) {
      framesDropped = 0;
    } else {
      framesDropped++;
      if (framesDropped > 5) {
        end = true;
      }
      return;
    }

    Translation3d translate = new Translation3d(0, 0, -2);
    translate = translate.rotateBy(pose.getRotation());
    translate = translate.plus(pose.getTranslation());
    SmartDashboard.putNumber("x", tagPoseRobot[0]);
    SmartDashboard.putNumber("y", tagPoseRobot[1]);
    SmartDashboard.putNumber("z", tagPoseRobot[2]);
    SmartDashboard.putNumber("rx", tagPoseRobot[3]); // pitch
    SmartDashboard.putNumber("ry", tagPoseRobot[4]); // yaw
    SmartDashboard.putNumber("rz", tagPoseRobot[5]); // roll
    SmartDashboard.putNumber("translated x", translate.getX());
    SmartDashboard.putNumber("translated y", translate.getY());
    SmartDashboard.putNumber("translated z", translate.getZ());
    
    double maxVelocity = 2;
    double xDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * translate.getZ()));
    double yDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * translate.getX()));
    SmartDashboard.putNumber("xDriveSpeed", xDriveSpeed);
    SmartDashboard.putNumber("yDriveSpeed", yDriveSpeed);
    
    driveSubsystem.setControl(drive.withVelocityX(-xDriveSpeed).withVelocityY(yDriveSpeed).withRotationalRate(LimelightHelpers.getTX("") * rotationalKP * rotationMult));

    // if (Math.abs(LimelightHelpers.getTX("")) < 5) {
    //   timer.start();
    // } else {
    //   timer.reset();
    // }

    // if (timer.get() > 0.5) {
    //   rotationMult = 0;
    // } else {
    //   rotationMult = 1;
    // }

    if (LimelightHelpers.getTX("") < 5 && Math.sqrt(Math.pow(translate.getZ(), 2) + Math.pow(translate.getX(), 2)) < 0.25){
      end = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
