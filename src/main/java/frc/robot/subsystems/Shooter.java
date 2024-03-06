// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  private final CANcoder armPositionEncoder = new CANcoder(18);
  private double targetArmPosition = 0;
  private double positionOfArm = 0;
  public static final double PIVOT_MOTOR_ANGLE_ERROR_THREASHOLD_ID = 1.0 / 360.0;

  public TalonFX getPivotMotor() {
    return pivotMotor;
  }

  public CANcoder getPivotMotorEncoder() {
    return armPositionEncoder;
  }

  public void shooterInit() {
    SmartDashboard.putNumber(SMART_DASHBOARD_VELOCITY, motorVelocity);
    SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_VELOCITY, targetVelocity);
    SmartDashboard.putNumber(SMART_DASHBOARD_POSITION, positionOfArm);
    SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_POSITION, targetArmPosition);

    leftShootMotor.setInverted(true);

    MagnetSensorConfigs wristPositionMagnetConfigs = new MagnetSensorConfigs();
    wristPositionMagnetConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    wristPositionMagnetConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    wristPositionMagnetConfigs.MagnetOffset = 0;  // TODO Calibrate this with a command
    armPositionEncoder.getConfigurator().apply(wristPositionMagnetConfigs);

    FeedbackConfigs pivotConfigs = new FeedbackConfigs();
    pivotConfigs.FeedbackRemoteSensorID = armPositionEncoder.getDeviceID();
    pivotConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotConfigs.RotorToSensorRatio = 45 / 8;
    pivotMotor.getConfigurator().apply(pivotConfigs);

    pivotMotor.getConfigurator().apply(ArmTunerConstants.pivotMotionMagicConfigs);
    pivotMotor.getConfigurator().apply(ArmTunerConstants.pivotPIDConfigs);

    TalonFX.optimizeBusUtilizationForAll(pivotMotor, leftShootMotor, rightShootMotor);
    stopMotors();
  }


  public void setAngle(double angle) {
    pivotMotor.setControl(motionMagicVoltage.withPosition(angle / 360));
    targetArmPosition = angle;
  }

  public void startMotors(double speed) {
    targetVelocity = speed;
    rightShootMotor.set(targetVelocity);
    leftShootMotor.set(targetVelocity);
  }

  public boolean canShoot() {
    return motorsAtShootingSpeed && Math.abs(pivotMotor.getClosedLoopError().getValue()) <= PIVOT_MOTOR_ANGLE_ERROR_THREASHOLD_ID;
  }

  public void readyArmForNewNote() {
    setAngle(30);
  }

  public void stopMotors() {
    rightShootMotor.set(0);
    leftShootMotor.set(0);
  }

  @Override
  public void periodic() {
    motorVelocity = leftShootMotor.getVelocity().getValue();
    motorsAtShootingSpeed = motorVelocity <= targetVelocity + 10 && motorVelocity >= targetVelocity - 10;
    positionOfArm = armPositionEncoder.getAbsolutePosition().getValue() * 360;

    SmartDashboard.putNumber(SMART_DASHBOARD_VELOCITY, motorVelocity);
    SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_VELOCITY, targetVelocity);
    SmartDashboard.putNumber(SMART_DASHBOARD_POSITION, positionOfArm);
    SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_POSITION, targetArmPosition);
  }
}
