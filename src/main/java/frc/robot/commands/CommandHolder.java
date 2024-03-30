package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.calibration.CalibrateWristAngleCommand;
import frc.robot.commands.intake.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class CommandHolder {
    private final Intake intake;
    private final Shooter shooter;
    private final GenericHID controller;

    private WaitForIntakeCommand waitForIntakeCommand;
    private IntakeNoteCommand intakeNoteCommand;
    private IntakeNoteCommand intakeNoteAndKeepRunningCommand;
    private MoveNoteToHoldingCommand moveNoteToHoldingCommand;
    private MoveNoteBackToShooterReadyCommand moveNoteBackToShooterReadyCommand;
    private HoldCommand holdCommand;
    private CalibrateWristAngleCommand calibrateWristAngleCommand;

    public CommandHolder(
            Intake intake,
            Shooter shooter,
            GenericHID controller
    ) {
        this.intake = intake;
        this.shooter = shooter;
        this.controller = controller;
    }

    public WaitForIntakeCommand waitForIntakeCommand() {
        if (waitForIntakeCommand == null) {
            waitForIntakeCommand = new WaitForIntakeCommand(this, intake);
        }
        return waitForIntakeCommand;
    }

    public IntakeNoteCommand intakeNoteCommand() {
        if (intakeNoteCommand == null) {
            intakeNoteCommand = new IntakeNoteCommand(this, intake, false);
        }
        return intakeNoteCommand;
    }

    public IntakeNoteCommand intakeNoteAndKeepRunningCommand() {
        if (intakeNoteAndKeepRunningCommand == null) {
            intakeNoteAndKeepRunningCommand = new IntakeNoteCommand(this, intake, true);
        }
        return intakeNoteAndKeepRunningCommand;
    }

    public MoveNoteToHoldingCommand moveNoteToHoldingCommand() {
        if (moveNoteToHoldingCommand == null) {
            moveNoteToHoldingCommand = new MoveNoteToHoldingCommand(this, intake, shooter);
        }
        return moveNoteToHoldingCommand;
    }

    public MoveNoteBackToShooterReadyCommand moveNoteBackToShooterReadyCommand() {
        if (moveNoteBackToShooterReadyCommand == null) {
            moveNoteBackToShooterReadyCommand = new MoveNoteBackToShooterReadyCommand(this, intake, shooter);
        }
        return moveNoteBackToShooterReadyCommand;
    }

    public HoldCommand holdCommand() {
        if (holdCommand == null) {
            holdCommand = new HoldCommand(intake);
        }
        return holdCommand;
    }

    public RumbleCommand rumbleCommand(RumbleCommand.Power power, RumbleCommand.Time time) {
        return new RumbleCommand(controller, power, time);
    }

    public CalibrateWristAngleCommand calibrateWristAngleCommand() {
        if(calibrateWristAngleCommand == null) {
            calibrateWristAngleCommand = new CalibrateWristAngleCommand(shooter);
        }

        return calibrateWristAngleCommand;
    }
}
