// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
    private final DigitalInput preIntakeSensor = new DigitalInput(7);
    private final DigitalInput intakeSensor = new DigitalInput(8);
    private final DigitalInput shooterSensor = new DigitalInput(9);

    private final TalonFX intakeMotor = new TalonFX(4, Constants.OperatorConstants.canivoreSerial);
    private final Servo RightIntakeServo = new Servo(1); // PWM channel is likely subject to change.
    private final Servo LeftIntakeServo = new Servo(2); // PWM channel is likely subject to change.

    private boolean status = intakeMotor.isAlive();

    private enum IntakeState {
        IDLE, // No motors are moving, no note is inside the mechanism
        INTAKING, // Motors are moving, note is not yet where we need it to be
        HOLDING, // No motors are moving, note is where it needs to be and is contained in the robot
        SHOOTING, // Motors are moving, note is being moved into the gears
    }

    private Timer timer = new Timer();
    private Timer postIntakeTimer = new Timer();

    private static IntakeState currentState = IntakeState.IDLE;

    public void intakeInit() {
        intakeMotor.setInverted(true);
        timer.stop();
        timer.reset();
        postIntakeTimer.stop();
        postIntakeTimer.reset();
    }

    public void optomizeCan() {
        TalonFX.optimizeBusUtilizationForAll(intakeMotor);
    }

    public void shoot(double speed) {
        LeftIntakeServo.setAngle(0); //Angle is subject to cahnge depending on the orientation of the servos.
        RightIntakeServo.setAngle(0); //Angle is subject to cahnge depending on the orientation of the servos.
        currentState = IntakeState.SHOOTING;        
        intakeMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Motor Functional?", status);
        SmartDashboard.putNumber("Intake Motor Speed", intakeMotor.get());
        SmartDashboard.putBoolean("Pre Intake Sensor", preIntakeSensor.get());
        SmartDashboard.putBoolean("Intake Sensor", intakeSensor.get());
        SmartDashboard.putBoolean("Shooter Sensor", shooterSensor.get());
        SmartDashboard.putString("Intake State", currentState.name());
        SmartDashboard.putNumber("Timer", timer.get());
        SmartDashboard.putNumber("Post Intake Timer", postIntakeTimer.get());
        boolean noteAtPreIntakeSensor = ! preIntakeSensor.get();
        boolean noteAtIntakeSensor = ! intakeSensor.get();
        boolean noteAtShooterSensor = ! shooterSensor.get();
        switch (currentState) {
            case IDLE:

                if (intakeMotor.get() != 0) {
                    intakeMotor.set(0);
                }

                if (noteAtPreIntakeSensor) {
                    LeftIntakeServo.setAngle(160);
                    RightIntakeServo.setAngle(0);

                    currentState = IntakeState.INTAKING;
                    timer.stop();
                    timer.reset();
                    postIntakeTimer.stop();
                    postIntakeTimer.reset();
                }

                break;
            case INTAKING:

                if (noteAtPreIntakeSensor || noteAtIntakeSensor || noteAtShooterSensor) { // If there is a note in the intake subsystem, keep the timer from starting and make sure the motor is moving
                    if (intakeMotor.get() == 0) {
                        intakeMotor.set(0.2);
                    }
                    timer.stop();
                    timer.reset();
                } else if (timer.get() == 0 && postIntakeTimer.get() != 0) { // If there is not a note in the subsystem, the timer has not started, and the note has not already passed the shooter sensor, start the timer to disable the motor
                    timer.start();
                }

                if (noteAtShooterSensor) { // Start the second timer when the note hits the shooter sensor
                    postIntakeTimer.reset();
                    postIntakeTimer.start();
                }

                if (postIntakeTimer.hasElapsed(0.1)) { // If the second timer has reached 0.1 seconds, we are now holding the note
                    currentState = IntakeState.HOLDING;
                    intakeMotor.stopMotor();
                    timer.stop();
                    timer.reset();
                    postIntakeTimer.stop();
                    postIntakeTimer.reset();
                }
                
                if (timer.hasElapsed(0.2)) { // If there has been no note detected inside the intake subsystem for 0.2 seconds, set back to idle and stop the motors
                    currentState = IntakeState.IDLE;
                    intakeMotor.stopMotor();
                    timer.stop();
                    timer.reset();
                    postIntakeTimer.stop();
                    postIntakeTimer.reset();
                }

                break;
            case HOLDING:

                LeftIntakeServo.setAngle(160);
                RightIntakeServo.setAngle(0);
                intakeMotor.stopMotor();

                break;
            case SHOOTING:

                if (timer.get() == 0) { // If the timer has not started, start the timer and the motor
                    intakeMotor.set(0.2);                    
                    timer.start();
                }

                if (timer.hasElapsed(1)) { // If the intake has been running for 1 second, the note has shot. Reset everything back to idle
                    currentState = IntakeState.IDLE;
                    intakeMotor.stopMotor();
                    timer.stop();
                    timer.reset();
                }

                break;
        }
    }

    public void disable() {
        intakeMotor.stopMotor();
        timer.stop();
        timer.reset();
        postIntakeTimer.stop();
        postIntakeTimer.reset();
    }
}