package frc.robot.commands.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

import java.util.function.Supplier;

public class WaitForIntakeCommand extends Command {
    private final Intake intake;
    private final Supplier<Boolean> noteAtPreIntakeSensor;
    private final Supplier<Boolean> noteAtIntakeSensor;

    private final NeutralOut brake = new NeutralOut();

    public WaitForIntakeCommand(
            Intake intake,
            Supplier<Boolean> noteAtPreIntakeSensor,
            Supplier<Boolean> noteAtIntakeSensor) {
        this.intake = intake;
        addRequirements(intake);

        this.noteAtPreIntakeSensor = noteAtPreIntakeSensor;
        this.noteAtIntakeSensor = noteAtIntakeSensor;
    }

    @Override
    public void initialize() {
        intake.setMotorControl(brake);
    }

    @Override
    public void execute() {
        boolean noteAtPreIntakeSensor = this.noteAtPreIntakeSensor.get();
        boolean noteAtIntakeSensor = this.noteAtIntakeSensor.get();

        // If there is a note at the intake, start intaking
        if (noteAtPreIntakeSensor || noteAtIntakeSensor) {
            intake.commands.intakeNote.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
