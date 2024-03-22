package frc.robot.commands.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class PassToShooterCommand extends Command {
    private final Timer timer = new Timer();

    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final NeutralOut brake = new NeutralOut();
    private final Intake intake;

    public PassToShooterCommand(
            Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setMotorControl(voltageOut.withOutput(2.4));

        timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotorControl(brake);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }
}
