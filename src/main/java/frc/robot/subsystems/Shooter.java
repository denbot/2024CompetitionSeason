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

public class Shooter extends SubsystemBase {

  private final static TalonFX pivotMotor = new TalonFX(14, Constants.OperatorConstants.canivoreSerial);
  private final TalonFX leftShootMotor = new TalonFX(5, Constants.OperatorConstants.canivoreSerial);
  private final TalonFX rightShootMotor = new TalonFX(13, Constants.OperatorConstants.canivoreSerial); 
  private double targetVelocity = 0;
  private double motorVelocity = 0;
  private boolean motorsAtShootingSpeed = false;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  private final String SMART_DASHBOARD_VELOCITY = "Shooter Motor Velocity";
  private final String SMART_DASHBOARD_TARGET_VELOCITY = "Shooter Motor Target Velocity";
  private final String SMART_DASHBOARD_POSITION = "Shooter Motor Position";
  private final String SMART_DASHBOARD_TARGET_POSITION = "Shooter Motor Target Position";

  private final static CANcoder wristPositionEncoder = new CANcoder(18);
  private double targetArmPosition = 0;
  private static double positionOfArm = 0;
  public static final double pivotMotorAngleErrorThreashhold = 1.0 / 360.0;

  public TalonFX getPivotMotor() {
    return pivotMotor;
  }

  public CANcoder getPivotMotorEncoder() {
    return wristPositionEncoder;
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
    wristPositionEncoder.getConfigurator().apply(wristPositionMagnetConfigs);

    FeedbackConfigs pivotConfigs = new FeedbackConfigs();
    pivotConfigs.FeedbackRemoteSensorID = wristPositionEncoder.getDeviceID();
    pivotConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotConfigs.RotorToSensorRatio = 45 / 8;
    pivotMotor.getConfigurator().apply(pivotConfigs);

    TalonFX.optimizeBusUtilizationForAll(pivotMotor, leftShootMotor, rightShootMotor);
    stopMotors();
  }

  public void optomizeCan() {
    TalonFX.optimizeBusUtilizationForAll(pivotMotor, leftShootMotor, rightShootMotor);
  }

  public void setAngle(double angle) {
    motionMagicVoltage = new MotionMagicVoltage(angle / 360);
    motionMagicVoltage = motionMagicVoltage.withOverrideBrakeDurNeutral(true);
    pivotMotor.setControl(motionMagicVoltage);
    targetArmPosition = angle;
  }

  public void startMotors(double speed) {
    targetVelocity = speed;
    rightShootMotor.set(targetVelocity);
    leftShootMotor.set(targetVelocity);
  }

  public boolean canShoot() {
    return motorsAtShootingSpeed && Math.abs(pivotMotor.getClosedLoopError().getValue()) <= pivotMotorAngleErrorThreashhold;
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
    positionOfArm = wristPositionEncoder.getAbsolutePosition().getValue() * 360;

    SmartDashboard.putNumber(SMART_DASHBOARD_VELOCITY, motorVelocity);
    SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_VELOCITY, targetVelocity);
    SmartDashboard.putNumber(SMART_DASHBOARD_POSITION, positionOfArm);
    SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_POSITION, targetArmPosition);
  }
}
