package frc.robot.commands.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class PassToShooterCommand extends Command {
    private final TalonFX intakeMotor;

    private final Timer timer = new Timer();

    public PassToShooterCommand(
            Intake intake,
            TalonFX intakeMotor) {
        addRequirements(intake);

        this.intakeMotor = intakeMotor;
    }

    @Override
    public void initialize() {
        intakeMotor.setVoltage(2.4);

        timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        intakeMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }
}
