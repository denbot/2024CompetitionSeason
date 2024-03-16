// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.calibration.CalibrateWristAngleCommand;
import frc.robot.generated.*;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.PrepCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter shooterSubsystem = new Shooter();
  private final Intake intakeSubsystem = new Intake();

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final ShootCommand shootCommand = new ShootCommand(shooterSubsystem, intakeSubsystem);
  
  private final PrepCommand firstShoot = new PrepCommand(shooterSubsystem, 30, 0.3); //TODO Change angle if necessary
  private final PrepCommand secondShoot = new PrepCommand(shooterSubsystem, 30, 0.3); //TODO Change angle if necessary
  private final PrepCommand stageSpeakerShoot = new PrepCommand(shooterSubsystem, 52.5, 0.9); //TODO Change angle if necessary
  private final PrepCommand trapShoot = new PrepCommand(shooterSubsystem, 66, 1); //TODO Change angle if necessary
  private final PrepCommand ampShoot = new PrepCommand(shooterSubsystem, 56, 0.25); //TODO Change angle if necessary
  private final PrepCommand speakerShoot = new PrepCommand(shooterSubsystem, 66, 0.75); //TODO Change angle if necessary

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private double maxSpeed = 6; // 6 meters per second desired top speed
  private double maxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveSubsystem drivetrain = SwerveTunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry telemetry = new Telemetry(maxSpeed);

  public RobotContainer() {
    configureBindings();
    intakeSubsystem.intakeInit();
    shooterSubsystem.shooterInit();

    NamedCommands.registerCommand("First Shoot", firstShoot);
    NamedCommands.registerCommand("Second Shoot", secondShoot);
    NamedCommands.registerCommand("Stage Speaker Shoot", stageSpeakerShoot);
    NamedCommands.registerCommand("Trap Shoot", trapShoot);

    //autoChooser = AutoBuilder.buildAutoChooser("");
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Uncomment this to calibrate the wrist angle
    // shooterSubsystem.setDefaultCommand(new CalibrateWristAngleCommand(shooterSubsystem));

    driverController.leftBumper().onTrue(ampShoot);
    driverController.rightBumper().onTrue(speakerShoot);
    driverController.x().onTrue(trapShoot);
    driverController.y().onTrue(stageSpeakerShoot);
    driverController.rightTrigger().onTrue(shootCommand);

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * maxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * maxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driverController.back().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.start().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on start button press
    driverController.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.zeroGyro()));


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));

    drivetrain.registerTelemetry(telemetry::telemeterize);
    }
  }

  public void setControllerVibrations() {
    // if we intake a note, it should only vibrate for less than a second
    if (shooterSubsystem.canShoot() || intakeSubsystem.intakedNote()) {
      driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
      return;
    }

    driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
