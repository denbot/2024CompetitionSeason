package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandHolder;
import frc.robot.subsystems.Intake;

public class MoveNoteToHoldingCommand extends Command {
    private final CommandHolder commands;
    private final Intake intake;

    public MoveNoteToHoldingCommand(
            CommandHolder commands,
            Intake intake
    ) {
        this.commands = commands;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        boolean noteAtShooterSensor = intake.noteAtShooterSensor();

        if(! noteAtShooterSensor) {  // We've moved the note past this sensor
            commands.moveNoteBackToShooterReadyCommand().schedule();
        }
    }
}
