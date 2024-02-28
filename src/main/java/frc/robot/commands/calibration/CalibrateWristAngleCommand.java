package frc.robot.commands.calibration;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class CalibrateWristAngleCommand extends Command {
    private static double ANGLE_0_TO_360_AT_WRIST_REST = 30.0;
    private static double ANGLE_0_TO_1_AT_WRIST_REST = ANGLE_0_TO_360_AT_WRIST_REST / 360;

    // private Shooter shooter;
    private final TalonFX pivotMotor;
    private final CANcoder wristPositionEncoder;
    private double currentProgrammedOffset;

    public CalibrateWristAngleCommand(Shooter shooter) {
        // this.shooter = shooter;
        this.pivotMotor = shooter.pivotMotor;
        this.wristPositionEncoder = shooter.wristPositionEncoder;

        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
        wristPositionEncoder.getConfigurator().refresh(sensorConfigs);
        currentProgrammedOffset = sensorConfigs.MagnetOffset;
        
        addRequirements(shooter);
    }

    public void execute() {
        double wristPosition = wristPositionEncoder.getAbsolutePosition().getValue();
        // double motorPosition = pivotMotor.getPosition().getValue();

        SmartDashboard.putNumber("Wrist Position", wristPosition);
        // SmartDashboard.putNumber("Motor Position", motorPosition);  // Useful if we do fused encoders, currently not implemented math wise

        /*
         * Spot on: ANGLE_0_TO_1_AT_WRIST_REST == wristPosition
         * ANGLE_0_TO_1_AT_WRIST_REST > wristPosition -> increase offset (positive number) ANGLE_0_TO_1_AT_WRIST_REST - wristPosition
         * ANGLE_0_TO_1_AT_WRIST_REST < wristPosition -> decrease offset (negative number) ANGLE_0_TO_1_AT_WRIST_REST - wristPosition
         */

        double wristOffsetWithCurrentProgrammedOffset = ANGLE_0_TO_1_AT_WRIST_REST - wristPosition;
        double wristEncoderOffsetAtZero = currentProgrammedOffset + wristOffsetWithCurrentProgrammedOffset;
        SmartDashboard.putNumber("New Wrist Zero offset", wristEncoderOffsetAtZero);
    }
}