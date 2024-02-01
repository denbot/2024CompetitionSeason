// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix6.hardware.TalonFX;


public class Shooter extends SubsystemBase {

  private final TalonFX pivotMotor = new TalonFX(0); // Set to pivot motor device Id
  private final TalonFX leftShootMotor = new TalonFX(0); // Set to left motor device Id
  private final TalonFX rightShootMotor = new TalonFX(0); // Set to left motor device Id
  private final TalonFX intakeMotor = new TalonFX(0); // Set to left motor device Id
  private double speed = 0;

  public Shooter() {}

  public void shooterInit() {
    SmartDashboard.putNumber("Shooter Motor Speed", 0);
    SmartDashboard.putString("Shooter Position", "Default");
  }

  public void startMotors(double speed, String position) {
    SmartDashboard.putNumber("Shooter Motor Speed", speed);
    SmartDashboard.putString("Shooter Position", position);
    rightShootMotor.set(speed);
    leftShootMotor.set(speed);
    this.speed = speed;
  }
  
  public void setPosition(int position) {
    pivotMotor.setcontrol(Diff_PositionDutyCycle_Position())
  }

  public void shoot(double speed) {
    if (this.speed == 0) {
      intakeMotor.set(speed);
    }
  }

  public void setDefault() {
    SmartDashboard.putNumber("Shooter Motor Speed", 0);
    SmartDashboard.putString("Shooter Position", "Default");
    intakeMotor.set(0);
    rightShootMotor.set(0);
    leftShootMotor.set(0);
    pivotMotor
    this.speed = speed;
  }


  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
