package frc.robot.commands.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class WaitForIntakeCommand extends Command {
    private final Intake intake;

    private final NeutralOut brake = new NeutralOut();

    public WaitForIntakeCommand(
            Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setMotorControl(brake);
    }

    @Override
    public void execute() {
        boolean noteAtPreIntakeSensor = intake.noteAtPreIntakeSensor();
        boolean noteAtIntakeSensor = intake.noteAtIntakeSensor();

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
