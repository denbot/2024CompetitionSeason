// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase {

  private final TalonFX pivotMotor = new TalonFX(0); // TODO: Set to pivot motor device Id
  private final TalonFX leftShootMotor = new TalonFX(0); // TODO: Set to left motor device Id
  private final TalonFX rightShootMotor = new TalonFX(0); // TODO: Set to left motor device Id
  private final TalonFX intakeMotor = new TalonFX(0); // TODO: Set to left motor device Id
  private double speed = 0;  // Switch to velocity
  private String position = "Default";
  public boolean canShoot = false;

  public Shooter() {}

  public void shooterInit() {
    SmartDashboard.putNumber("Shooter Motor Speed", 0);
    SmartDashboard.putString("Shooter Position", "Default");
    leftShootMotor.setInverted(true); // TODO: Change to Right/Left to invert shooting motor
  }

  public void startMotors(double speed, String position) {
    this.speed = speed;
    this.position = position;
    SmartDashboard.putNumber("Shooter Motor Speed", speed);
    SmartDashboard.putString("Shooter Position", position);
    if (position == "Amp") {
      pivotMotor.setPosition(0.2);  // TODO: Change to fit amp angle
    } else if (position == "Speaker") {
      pivotMotor.setPosition(0.1);  // TODO: Change to fit speaker angle
    }
    rightShootMotor.set(speed);  // Switch to velocity
    leftShootMotor.set(speed);  // Switch to velocity
    canShoot = true;
  }

  public void shoot(double speed) {
    if (this.speed != 0 && (this.position == "Default")) { // If shooter is in position and ready to fire
      intakeMotor.set(speed);  // Switch to velocity
    }
    canShoot = false;
  }

  public void setDefault() {
    SmartDashboard.putNumber("Shooter Motor Speed", 0);
    SmartDashboard.putString("Shooter Position", "Default");
    intakeMotor.set(0);  // Switch to velocity
    rightShootMotor.set(0);  // Switch to velocity
    leftShootMotor.set(0);  // Switch to velocity
    if (this.position == "Amp") {
      pivotMotor.setPosition(-0.2);  // TODO: Change to fit amp angle
    } else if (this.position == "Speaker") {
      pivotMotor.setPosition(-0.1);  // TODO: Change to fit speaker angle
    }
    this.position = "Default";
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
