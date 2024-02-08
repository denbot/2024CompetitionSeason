// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterTest extends SubsystemBase {
  /** Creates a new ShooterTest. */
  private double shooterSpeed = 0.0;
  private VelocityVoltage velocity = new VelocityVoltage(shooterSpeed);
  private TalonFX happyMotor2 = new TalonFX(0);
  @Override
  public void periodic() {
    happyMotor2.setControl(velocity.withVelocity(shooterSpeed));
  }
}
