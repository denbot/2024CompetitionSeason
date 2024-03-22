package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

import java.util.function.Supplier;

public class MoveNoteToHoldingCommand extends Command {
    private final Intake intake;
    private final Supplier<Boolean> noteAtShooterSensor;

    public MoveNoteToHoldingCommand(
            Intake intake,
            Supplier<Boolean> noteAtShooterSensor) {
        this.intake = intake;
        addRequirements(intake);

        this.noteAtShooterSensor = noteAtShooterSensor;
    }

    @Override
    public void execute() {
        boolean noteAtShooterSensor = this.noteAtShooterSensor.get();

        if(! noteAtShooterSensor) {  // We've moved the note past this sensor
            intake.commands.moveNoteBackToShooterReady.schedule();
        }
    }
}
