package frc.robot.commands.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class MoveNoteBackToShooterReadyCommand extends Command {
    private final Intake intake;

    private final Timer timer = new Timer();

    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final NeutralOut brake = new NeutralOut();

    public MoveNoteBackToShooterReadyCommand(
            Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.restart();
        intake.setMotorControl(voltageOut.withOutput(-1.2));  // Move the note back slightly
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotorControl(brake);
        intake.commands.hold.schedule();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.1);
    }
}
