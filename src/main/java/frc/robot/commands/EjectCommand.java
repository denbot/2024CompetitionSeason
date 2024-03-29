package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

// ejects note out of intake
// use when we have accidently intaked 2+ notes, which we can not deal with
public class EjectCommand extends Command {
    private final Intake intake;

    public EjectCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.eject(1.2); // may need to be tuned to whatever the voltage is to eject quickly
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopEject();
    }

    @Override
    public boolean isFinished() {
        return !intake.noteInIntake();
    }
}
