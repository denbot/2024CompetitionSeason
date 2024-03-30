package frc.robot.commands.calibration;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class CalibrateWristAngleCommand extends Command {
    private static final double ANGLE_0_TO_360_AT_WRIST_REST = 29.6;
    private static final double ANGLE_0_TO_1_AT_WRIST_REST = ANGLE_0_TO_360_AT_WRIST_REST / 360;

    private final Shooter shooter;
    private final CANcoder wristPositionEncoder;
    private final TalonFX pivotMotor;
    private double currentProgrammedOffset;
    private double wristEncoderOffsetAtZero;

    public CalibrateWristAngleCommand(Shooter shooter) {
        this.shooter = shooter;
        this.wristPositionEncoder = shooter.getPivotMotorEncoder();
        this.pivotMotor = shooter.getPivotMotor();

        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
        wristPositionEncoder.getConfigurator().refresh(sensorConfigs);
        currentProgrammedOffset = sensorConfigs.MagnetOffset;

        addRequirements(shooter);
    }

    public void execute() {
        double wristPosition = wristPositionEncoder.getAbsolutePosition().getValue();

        SmartDashboard.putNumber("Wrist Position", wristPosition * 360);

        /*
         * Spot on: ANGLE_0_TO_1_AT_WRIST_REST == wristPosition
         * ANGLE_0_TO_1_AT_WRIST_REST > wristPosition -> increase offset (positive number) ANGLE_0_TO_1_AT_WRIST_REST - wristPosition
         * ANGLE_0_TO_1_AT_WRIST_REST < wristPosition -> decrease offset (negative number) ANGLE_0_TO_1_AT_WRIST_REST - wristPosition
         */

        double wristOffsetWithCurrentProgrammedOffset = ANGLE_0_TO_1_AT_WRIST_REST - wristPosition;
        wristEncoderOffsetAtZero = currentProgrammedOffset + wristOffsetWithCurrentProgrammedOffset;
        SmartDashboard.putNumber("New Wrist Zero offset", wristEncoderOffsetAtZero);
    }

    public void recalibrate() {
        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
        currentProgrammedOffset = sensorConfigs.MagnetOffset = wristEncoderOffsetAtZero;
        wristPositionEncoder.getConfigurator().refresh(sensorConfigs);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
