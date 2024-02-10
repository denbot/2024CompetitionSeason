// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
 
public class Intake extends SubsystemBase{
    private final DigitalInput preIntakeSensor = new DigitalInput(0);
    private final DigitalInput intakeSensor = new DigitalInput(1);
    private final DigitalInput shooterSensor = new DigitalInput(2);

    private final TalonFX intakeMotor = new TalonFX(0);

    boolean status = intakeMotor.isAlive();
    //Will change intakeMotorVelocity
    double intakeMotorVelocity = 1;
    private VelocityVoltage velocity = new VelocityVoltage(intakeMotorVelocity);

    enum IntakeState {
        IDLE,  // No motors are moving, no note is inside the mechanism
        INTAKING, // Motors are moving, note is not yet where we need it to be
        HOLDING, // No motors are moving, note is where it needs to be and is contained in the robot
        EJECTING;  // Motors are moving, note is being moved into the gears
    }
    public static IntakeState CurrentState = IntakeState.IDLE;
    
    public Intake(){
        // SmartDashboard displays to test sensors
        SmartDashboard.putBoolean("Motor Functional?", status);
        SmartDashboard.putNumber("Intake Motor Speed (RPM)", intakeMotorVelocity);
    }

    @Override
    public void periodic(){
        switch (CurrentState) {
            case IDLE:
                if (preIntakeSensor.get() == false){ 
                    intakeMotor.setControl(velocity.withVelocity(intakeMotorVelocity));
                    CurrentState = IntakeState.INTAKING;
                }
                    break;
                    case INTAKING:
                    if (preIntakeSensor.get() == true && intakeSensor.get() == true){
                        CurrentState = IntakeState.IDLE;
                    }
                    if (shooterSensor.get() == false){
                        CurrentState = IntakeState.HOLDING;
                }
                    break;
            case HOLDING:
                intakeMotor.set(0);
                    break;
            case EJECTING:
                while(shooterSensor.get() == false){
                    intakeMotor.setControl(velocity.withVelocity(intakeMotorVelocity));
                }
                intakeMotor.set(0);
                CurrentState = IntakeState.IDLE;
                    break;
        }
    }
    
    // Called on by shooter subsystem
    public void shoot(){
        CurrentState = IntakeState.EJECTING;
    }

    public void disable(){
        intakeMotor.set(0);
    }
}