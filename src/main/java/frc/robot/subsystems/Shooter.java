// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Shooter extends SubsystemBase {

  private final TalonFX pivotMotor = new TalonFX(0); // TODO: Set to pivot motor device Id
  private final TalonFX leftShootMotor = new TalonFX(0); // TODO: Set to left motor device Id
  private final TalonFX rightShootMotor = new TalonFX(0); // TODO: Set to right motor device Id
  private double speed = 0;
  private VelocityVoltage velocity = new VelocityVoltage(speed);
  public static boolean motorsAtShootingSpeed = false;
  private final String smartDashboardSpeed = "Shooter Motor Speed";
  private final String smartDashboardPosition = "Shooter Motor Speed";

  public enum position {
    DEFAULT,
    AMP,
    SPEAKER;
  }

  public static position positionOfArm = position.DEFAULT;

  public Shooter() {}

  public void shooterInit() {
    SmartDashboard.putNumber(smartDashboardSpeed, speed);
    SmartDashboard.putString(smartDashboardPosition, positionOfArm.toString());
    leftShootMotor.setInverted(true); // TODO: Change to Right/Left to invert shooting motor
  }

  public void startMotors(double speed, position position) {
    this.speed = speed;
    positionOfArm = position;
    SmartDashboard.putNumber(smartDashboardSpeed, speed);
    SmartDashboard.putString(smartDashboardPosition, positionOfArm.toString());
    if (position == position.AMP) {
      pivotMotor.setPosition(-0.05);  // TODO: Change to fit amp angle
    } else if (position == position.SPEAKER) {
      pivotMotor.setPosition(-0.1);  // TODO: Change to fit speaker angle
    }
    rightShootMotor.setControl(velocity.withVelocity(speed));
    leftShootMotor.setControl(velocity.withVelocity(speed));
  }

  public void shoot() {
    if (motorsAtShootingSpeed == true) { // If shooter is in position and ready to fire
      motorsAtShootingSpeed = false;
    }
  }

  public void setDefault() {
    SmartDashboard.putNumber(smartDashboardSpeed, 0);
    SmartDashboard.putString(smartDashboardPosition, positionOfArm.toString());
    rightShootMotor.setControl(velocity.withVelocity(0));
    leftShootMotor.setControl(velocity.withVelocity(0));
    if (positionOfArm == position.AMP) {
      pivotMotor.setPosition(-0.1);  // TODO: Change to fit amp angle
    } else if (positionOfArm == position.SPEAKER) {
      pivotMotor.setPosition(-0.05);  // TODO: Change to fit speaker angle
    }
    positionOfArm = position.DEFAULT;
  }

  @Override
  public void periodic() {
    if (leftShootMotor.getVelocity().refresh().equals(Math.round(speed/25)*25) && positionOfArm != position.DEFAULT) { // This will effectively give a range of 25 RPM for comparing
      motorsAtShootingSpeed = true;
    }
  }
}