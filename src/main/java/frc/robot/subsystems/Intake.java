// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
    private final DigitalInput preIntakeSensor = new DigitalInput(7);
    private final DigitalInput intakeSensor = new DigitalInput(8);
    private final DigitalInput shooterSensor = new DigitalInput(9);

    private final TalonFX intakeMotor = new TalonFX(4, Constants.OperatorConstants.canivoreSerial);

    private boolean notePassedShooterSensor = false;
    private boolean noteHitShooter = false;
    public boolean noteReady = false;

    private boolean status = intakeMotor.isAlive();

    private enum IntakeState {
        IDLE, // No motors are moving, no note is inside the mechanism
        INTAKING, // Motors are moving, note is not yet where we need it to be
        HOLDING, // No motors are moving, note is where it needs to be and is contained in the robot
        SHOOTING, // Motors are moving, note is being moved into the gears
    }

    private Timer timer = new Timer();


    public void intakeInit() {
        intakeMotor.setInverted(true);
        timer.stop();
        timer.reset();
    }


    public void optomizeCan() {
        TalonFX.optimizeBusUtilizationForAll(intakeMotor);
    }

    public void shoot(double volts) {
        intakeMotor.setVoltage(volts);
    }

    public boolean intakedNote() {
        return !intakeSensor.get();
    }

    public void setIntakeSpeed(double volts) {
        intakeMotor.setVoltage(volts);
    }

    public boolean noteInIntake() {
        return !preIntakeSensor.get() || !intakeSensor.get() || !shooterSensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Motor Functional?", status);
        SmartDashboard.putNumber("Intake Motor Speed", intakeMotor.get());
        SmartDashboard.putBoolean("Pre Intake Sensor", preIntakeSensor.get());
        SmartDashboard.putBoolean("Intake Sensor", intakeSensor.get());
        SmartDashboard.putBoolean("Shooter Sensor", shooterSensor.get());
    }

    public void disable() {
        intakeMotor.stopMotor();
        timer.stop();
        timer.reset();
    }
}
