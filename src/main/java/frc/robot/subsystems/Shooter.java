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

  public final TalonFX pivotMotor = new TalonFX(14, Constants.OperatorConstants.canivoreSerial);
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

  public CANcoder wristPositionEncoder = new CANcoder(18);
  private double targetArmPosition = 0;

  public enum Position {
    DEFAULT,
    AMP,
    SPEAKER;
  }

  private static Position positionOfArm = Position.DEFAULT;

  public void shooterInit() {
    SmartDashboard.putNumber(SMART_DASHBOARD_VELOCITY, motorVelocity);
    SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_VELOCITY, targetVelocity);
    SmartDashboard.putString(SMART_DASHBOARD_POSITION, positionOfArm.toString());
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

  public void startMotors(double speed, Position position) {
    targetVelocity = speed;
    positionOfArm = position;
    if (position == Position.AMP) {
      motionMagicVoltage = new MotionMagicVoltage(0.1);
      motionMagicVoltage = motionMagicVoltage.withOverrideBrakeDurNeutral(true);
      pivotMotor.setControl(motionMagicVoltage);
    } else if (position == Position.SPEAKER) {
      motionMagicVoltage = new MotionMagicVoltage(0.2);
      motionMagicVoltage = motionMagicVoltage.withOverrideBrakeDurNeutral(true);
      pivotMotor.setControl(motionMagicVoltage);
    }
    rightShootMotor.set(targetVelocity);
    leftShootMotor.set(targetVelocity);
  }

  public boolean canShoot() {
    return motorsAtShootingSpeed && positionOfArm != Position.DEFAULT;
  }

  public void setDefault() {
    if (positionOfArm == Position.AMP) {
      motionMagicVoltage = new MotionMagicVoltage(0);
      motionMagicVoltage = motionMagicVoltage.withOverrideBrakeDurNeutral(true);
      pivotMotor.setControl(motionMagicVoltage);
    } else if (positionOfArm == Position.SPEAKER) {
      motionMagicVoltage = new MotionMagicVoltage(0);
      motionMagicVoltage = motionMagicVoltage.withOverrideBrakeDurNeutral(true);
      pivotMotor.setControl(motionMagicVoltage);
    }
    positionOfArm = Position.DEFAULT;
  }

  public void stopMotors() {
    rightShootMotor.set(0);
    leftShootMotor.set(0);
  }

  @Override
  public void periodic() {
    motorVelocity = leftShootMotor.getVelocity().getValue();
    motorsAtShootingSpeed = motorVelocity <= targetVelocity + 10 && motorVelocity >= targetVelocity - 10;
    SmartDashboard.putNumber(SMART_DASHBOARD_VELOCITY, motorVelocity);
    SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_VELOCITY, targetVelocity);
    SmartDashboard.putString(SMART_DASHBOARD_POSITION, positionOfArm.toString());
    SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_POSITION, targetArmPosition);
    pivotMotor.setControl(motionMagicVoltage); // -- not controled yet
  }
}