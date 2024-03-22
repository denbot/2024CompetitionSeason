package frc.robot.commands.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class MoveNoteBackToShooterReadyCommand extends Command {
    private final Intake intake;
    private final TalonFX intakeMotor;

    private final Timer timer = new Timer();

    public MoveNoteBackToShooterReadyCommand(
            Intake intake,
            TalonFX intakeMotor) {
        this.intake = intake;
        addRequirements(intake);

        this.intakeMotor = intakeMotor;
    }

    @Override
    public void initialize() {
        timer.restart();
        intakeMotor.setVoltage(-1.2);  // Move the note back slightly
    }

    @Override
    public void end(boolean interrupted) {
        intakeMotor.stopMotor();
        intake.commands.hold.schedule();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.1);
    }
}
