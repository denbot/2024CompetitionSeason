// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Shooter extends SubsystemBase {

  private final TalonFX pivotMotor = new TalonFX(14)
  private final TalonFX leftShootMotor = new TalonFX(5);
  private final TalonFX rightShootMotor = new TalonFX(13);
  private double targetVelocity = 0;
  private double motorVelocity = 0;
  private boolean motorsAtShootingSpeed = false;
  private VelocityVoltage velocity = new VelocityVoltage(targetVelocity);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.1);
  private final String SMART_DASHBOARD_VELOCITY = "Shooter Motor Velocity";
  private final String SMART_DASHBOARD_TARGET_VELOCITY = "Shooter Motor Target Velocity";
  private final String SMART_DASHBOARD_POSITION = "Shooter Motor Position";

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
    leftShootMotor.setInverted(true); // TODO: Change to Right/Left to invert shooting motor
  }

  public void startMotors(double speed, Position position) {
    targetVelocity = speed;
    positionOfArm = position;
    if (position == Position.AMP) {
      motionMagicVoltage = new MotionMagicVoltage(-0.05);  // TODO: Change to fit amp angle -- not controled yet
    } else if (position == Position.SPEAKER) {
      motionMagicVoltage = new MotionMagicVoltage(-0.1);  // TODO: Change to fit speaker angle -- not controled yet
    }
    rightShootMotor.setControl(velocity.withVelocity(speed));
    leftShootMotor.setControl(velocity.withVelocity(speed));
  }

  public boolean canShoot() {
    return motorsAtShootingSpeed && positionOfArm != Position.DEFAULT;
  }

  public void setDefault() {
    if (positionOfArm == Position.AMP) {
      motionMagicVoltage = new MotionMagicVoltage(0.05);  // TODO: Change to fit amp angle -- not controled yet
    } else if (positionOfArm == Position.SPEAKER) {
      motionMagicVoltage = new MotionMagicVoltage(0.1);  // TODO: Change to fit speaker angle -- not controled yet
    }
    positionOfArm = Position.DEFAULT;
  }

  public void stopMotors() {
    rightShootMotor.setControl(velocity.withVelocity(0));
    leftShootMotor.setControl(velocity.withVelocity(0));
  }

  @Override
  public void periodic() {
    motorVelocity = leftShootMotor.getVelocity().getValue();
    motorsAtShootingSpeed = motorVelocity <= targetVelocity + 10 && motorVelocity >= targetVelocity - 10;
    SmartDashboard.putNumber(SMART_DASHBOARD_VELOCITY, motorVelocity);
    SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_VELOCITY, targetVelocity);
    SmartDashboard.putString(SMART_DASHBOARD_POSITION, positionOfArm.toString());
    pivotMotor.setControl(motionMagicVoltage); // -- not controled yet
  }
}