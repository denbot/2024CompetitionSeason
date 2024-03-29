// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MechanicalConstants;
import frc.robot.generated.ArmTunerConstants;

public class Shooter extends SubsystemBase {

    private final TalonFX pivotMotor = new TalonFX(14, Constants.OperatorConstants.canivoreSerial);
    private final TalonFX leftShootMotor = new TalonFX(5, Constants.OperatorConstants.canivoreSerial);
    private final TalonFX rightShootMotor = new TalonFX(13, Constants.OperatorConstants.canivoreSerial);
    private double targetVelocity = 0;
    private double motorVelocity = 0;
    private boolean motorsAtShootingSpeed = false;
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0)
            .withOverrideBrakeDurNeutral(true);


    private final String SMART_DASHBOARD_VELOCITY = "Shooter Motor Velocity";
    private final String SMART_DASHBOARD_TARGET_VELOCITY = "Shooter Motor Target Velocity";
    private final String SMART_DASHBOARD_POSITION = "Shooter Motor Position";
    private final String SMART_DASHBOARD_TARGET_POSITION = "Shooter Motor Target Position";

    private final CANcoder armPositionEncoder = new CANcoder(18, Constants.OperatorConstants.canivoreSerial);
    private double targetArmPosition = 40;
    private double positionOfArm = 0;
    public static final double PIVOT_MOTOR_ANGLE_ERROR_THREASHOLD_ID = 1.0 / 360.0;
    private final NeutralOut brake = new NeutralOut();

    private VelocityVoltage shooterControl = new VelocityVoltage(0).withEnableFOC(true);


    public CANcoder getPivotMotorEncoder() {
        return armPositionEncoder;
    }

    public void shooterInit() {
        SmartDashboard.putNumber(SMART_DASHBOARD_VELOCITY, motorVelocity);
        SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_VELOCITY, targetVelocity);
        SmartDashboard.putNumber(SMART_DASHBOARD_POSITION, positionOfArm);
        SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_POSITION, targetArmPosition);

        leftShootMotor.setNeutralMode(NeutralModeValue.Brake);
        rightShootMotor.setNeutralMode(NeutralModeValue.Brake);

        leftShootMotor.setInverted(true);

        MagnetSensorConfigs wristPositionMagnetConfigs = new MagnetSensorConfigs();
        wristPositionMagnetConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        wristPositionMagnetConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        wristPositionMagnetConfigs.MagnetOffset = 0.73933;  // Calibrate this with CalibrateWristAngleCommand
        armPositionEncoder.getConfigurator().apply(wristPositionMagnetConfigs);

        FeedbackConfigs pivotConfigs = new FeedbackConfigs();
        pivotConfigs.FeedbackRemoteSensorID = armPositionEncoder.getDeviceID();
        pivotConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfigs.RotorToSensorRatio = 45.0 / 8.0;
        pivotMotor.getConfigurator().apply(pivotConfigs);

        pivotMotor.getConfigurator().apply(ArmTunerConstants.pivotMotionMagicConfigs);
        pivotMotor.getConfigurator().apply(ArmTunerConstants.pivotPIDConfigs);

        SoftwareLimitSwitchConfigs pivotLimits = new SoftwareLimitSwitchConfigs();
        pivotLimits.ForwardSoftLimitThreshold = MechanicalConstants.maxArmAngle / 360.0;
        pivotLimits.ReverseSoftLimitThreshold = MechanicalConstants.minArmAngle / 360.0;
        pivotLimits.ForwardSoftLimitEnable = true;
        pivotLimits.ReverseSoftLimitEnable = true;
        pivotMotor.getConfigurator().apply(pivotLimits);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotor.getConfigurator().apply(outputConfigs);
        leftShootMotor.getConfigurator().apply(ArmTunerConstants.shooterPIDConfigs);
        rightShootMotor.getConfigurator().apply(ArmTunerConstants.shooterPIDConfigs);

        armPositionEncoder.getAbsolutePosition().setUpdateFrequency(200);
        leftShootMotor.getVelocity().setUpdateFrequency(50);
        rightShootMotor.getVelocity().setUpdateFrequency(50);
//    TalonFX.optimizeBusUtilizationForAll(pivotMotor, leftShootMotor, rightShootMotor);
        stopMotors();
        setAngle(targetArmPosition);
    }

    public void setAngle(double angle) {
        pivotMotor.setControl(new PositionVoltage(angle / 360.0));
//    pivotMotor.setControl(motionMagicVoltage.withPosition(angle / 360));
        targetArmPosition = angle;
    }

    public void startMotors(double rotationsPerSecond) {
        VelocityVoltage vspeed = shooterControl.withVelocity(rotationsPerSecond);
        targetVelocity = rotationsPerSecond;
        motorsAtShootingSpeed = true;
        rightShootMotor.setControl(vspeed);
        leftShootMotor.setControl(vspeed);

    }

    public boolean canShoot() {
        // return ((Math.abs(pivotMotor.getClosedLoopError().getValue()) <= PIVOT_MOTOR_ANGLE_ERROR_THREASHOLD_ID) && motorsAtShootingSpeed);
        return true;
    }

    public void readyArmForNewNote() {
        setAngle(30);
    }

    public void intake(double speed) {
        rightShootMotor.setVoltage(speed);
        leftShootMotor.setVoltage(speed);
    }

    public void stopMotors() {
        motorsAtShootingSpeed = false;
        rightShootMotor.setControl(brake);
        leftShootMotor.setControl(brake);
    }

    public void setVolts(double volts) {
        rightShootMotor.setControl(new VoltageOut(volts));
        leftShootMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void periodic() {
        motorVelocity = leftShootMotor.getVelocity().getValue();
        positionOfArm = armPositionEncoder.getAbsolutePosition().getValue() * 360;
        double pivotError = pivotMotor.getClosedLoopError().getValue() * 360;

        SmartDashboard.putNumber(SMART_DASHBOARD_VELOCITY, motorVelocity);
        SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_VELOCITY, targetVelocity);
        SmartDashboard.putNumber(SMART_DASHBOARD_POSITION, positionOfArm);
        SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_POSITION, targetArmPosition);
        SmartDashboard.putNumber("Arm position rotations", armPositionEncoder.getPosition().getValue());
        SmartDashboard.putNumber("Wrist Error", pivotError);
    }
}
