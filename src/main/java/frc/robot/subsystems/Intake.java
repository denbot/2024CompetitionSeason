// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PWM;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
    private final DigitalInput preIntakeSensor = new DigitalInput(7);
    private final DigitalInput intakeSensor = new DigitalInput(8);
    private final DigitalInput shooterSensor = new DigitalInput(9);

    private final TalonFX intakeMotor = new TalonFX(4, Constants.OperatorConstants.canivoreSerial);
    private final Servo RightIntakeServo = new Servo(1); //PWM channel is likely subject to change.
    private final Servo LeftIntakeServo = new Servo(2); //PWM channel is likely subject to change.


    private boolean status = intakeMotor.isAlive();
    private double intakeMotorVelocity = 4;
    private VelocityVoltage velocity = new VelocityVoltage(intakeMotorVelocity);
    private boolean isEjecting = false;

    private enum IntakeState {
        IDLE, // No motors are moving, no note is inside the mechanism
        INTAKING, // Motors are moving, note is not yet where we need it to be
        HOLDING, // No motors are moving, note is where it needs to be and is contained in the robot
        EJECTING; // Motors are moving, note is being moved into the gears
    }
    private Timer timer = new Timer();
    private Timer timerTwoElectrickBoogaloo = new Timer();
    

    private static IntakeState currentState = IntakeState.IDLE;

    public void intakeInit() {
        timerTwoElectrickBoogaloo.stop();
        timerTwoElectrickBoogaloo.reset();
        timer.stop();
        timer.reset();
        intakeMotor.setInverted(true);
    }

    public void optomizeCan() {
        TalonFX.optimizeBusUtilizationForAll(intakeMotor);
    }

    public void eject(double speed) {
        intakeMotor.set(speed);
        isEjecting = true;
        switch (currentState) {
            case HOLDING:
                currentState = IntakeState.IDLE;
        }
    }

    public void shoot(double speed) {
        LeftIntakeServo.setAngle(0); //Angle is subject to cahnge depending on the orientation of the servos.
        RightIntakeServo.setAngle(0); //Angle is subject to cahnge depending on the orientation of the servos.
        intakeMotor.set(speed);
        currentState = IntakeState.EJECTING;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Motor Functional?", status);
        SmartDashboard.putNumber("Intake Motor Speed (RPM)", intakeMotorVelocity);
        SmartDashboard.putBoolean("Pre Intake Sensor", preIntakeSensor.get());
        SmartDashboard.putString("Intake State", currentState.name());
        SmartDashboard.putNumber("Timer", timer.get());
        boolean noteAtPreIntakeSensor = ! preIntakeSensor.get();
        boolean noteAtIntakeSensor = ! intakeSensor.get();
        boolean noteAtShooterSensor = ! shooterSensor.get();
        switch (currentState) {
            case IDLE:
                if (noteAtPreIntakeSensor || noteAtIntakeSensor) {
                    RightIntakeServo.setAngle(180); //Angle is subject to change depending on the orientation of the servos.
                    LeftIntakeServo.setAngle(0); //Angle is subject to change depending on the orientation of the servos.
                    if (! isEjecting) {
                        intakeMotor.set(0.2);
                    }
                    currentState = IntakeState.INTAKING;
                    timer.stop();
                    timer.reset();
                }
                break;
            case INTAKING:
                if (noteAtPreIntakeSensor || noteAtIntakeSensor) {
                    if(timer.get() != 0) {
                        timer.stop();
                        timer.reset();
                    }
                } else {
                    // Our note is no at either intake sensor, start our timer to stop our motors
                    if(timer.get() == 0) {
                        timer.start();
                    }
                }

                if (noteAtShooterSensor && ! isEjecting) {
                    timer.stop();
                    timerTwoElectrickBoogaloo.start();
                    if (timerTwoElectrickBoogaloo.hasElapsed(0.1)) {
                        intakeMotor.set(0);
                        timerTwoElectrickBoogaloo.stop();
                        timerTwoElectrickBoogaloo.reset();
                        currentState = IntakeState.INTAKING;
                    }
                }

                if (timer.hasElapsed(0.2)) {
                    currentState = IntakeState.IDLE;
                    timer.stop();
                    timer.reset();
                    intakeMotor.set(0);
                    isEjecting = false;
                }
                break;
            case HOLDING:
                LeftIntakeServo.setAngle(120); //Angle is subject to change depending on the orientation of the servos.
                RightIntakeServo.setAngle(60); //Angle is subject to change depending on the orientation of the servos.
                intakeMotor.stopMotor();
                break;
            case EJECTING:;
                if (! noteAtShooterSensor) {
                    intakeMotor.stopMotor();
                    currentState = IntakeState.IDLE;
                }
                break;
        }
    }

    // Called on by shooter subsystem
    public void shoot() {
        currentState = IntakeState.EJECTING;
    }

    public void disable() {
        intakeMotor.stopMotor();
    }
}