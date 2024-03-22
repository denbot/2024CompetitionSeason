package frc.robot.commands.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

import java.util.function.Supplier;

public class IntakeNoteCommand extends Command {
    private final Intake intake;
    private final Supplier<Boolean> noteAtPreIntakeSensor;
    private final Supplier<Boolean> noteAtIntakeSensor;
    private final Supplier<Boolean> noteAtShooterSensor;

    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final NeutralOut brake = new NeutralOut();

    public IntakeNoteCommand(
            Intake intake,
            Supplier<Boolean> noteAtPreIntakeSensor,
            Supplier<Boolean> noteAtIntakeSensor,
            Supplier<Boolean> noteAtShooterSensor) {
        this.intake = intake;
        addRequirements(intake);

        this.noteAtPreIntakeSensor = noteAtPreIntakeSensor;
        this.noteAtIntakeSensor = noteAtIntakeSensor;
        this.noteAtShooterSensor = noteAtShooterSensor;
    }

    @Override
    public void initialize() {
        intake.setMotorControl(voltageOut.withOutput(2.4));  // Chosen by fair dice roll
    }

    @Override
    public void execute() {
        boolean noteAtPreIntakeSensor = this.noteAtPreIntakeSensor.get();
        boolean noteAtIntakeSensor = this.noteAtIntakeSensor.get();
        boolean noteAtShooterSensor = this.noteAtShooterSensor.get();
        boolean noteAtAnySensor = noteAtPreIntakeSensor || noteAtIntakeSensor || noteAtShooterSensor;

        // If none of our sensors are tripped, stop our intake motor. We lost the note
        if(! noteAtAnySensor) {
            intake.setMotorControl(brake);
            this.cancel();
        } else if (noteAtShooterSensor) {  // Now we know when the note hits the shooter sensor
            intake.commands.moveNoteToHolding.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
