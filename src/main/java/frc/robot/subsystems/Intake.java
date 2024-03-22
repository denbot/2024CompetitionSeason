// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private static IntakeState currentState = IntakeState.IDLE;


    public void intakeInit() {
        intakeMotor.setInverted(true);
        timer.stop();
        timer.reset();
    }


    public void optomizeCan() {
        TalonFX.optimizeBusUtilizationForAll(intakeMotor);
    }


    public void shoot(double volts) {
        currentState = IntakeState.SHOOTING;
        intakeMotor.setVoltage(volts);
    }

    /*** @param volts volts to apply to intakemotor (should be positive)
     ***/
    public Command eject(double volts) {
        // if (!noteInIntake() || currentState != IntakeState.HOLDING) {
        //     return;
        // }

        return Commands.run(() -> intakeMotor.setVoltage(-volts));
    }

    public void stopEject() {
        intakeMotor.setVoltage(0);
    }

    public boolean noteInIntake() {
        return !preIntakeSensor.get() || !intakeSensor.get() || !shooterSensor.get();
    }

    public boolean intakedNote() {
        return !intakeSensor.get();
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
        boolean noteAtPreIntakeSensor = !preIntakeSensor.get();
        boolean noteAtIntakeSensor = !intakeSensor.get();
        boolean noteAtShooterSensor = !shooterSensor.get();
        switch (currentState) {
            case IDLE:

                if (intakeMotor.get() != 0) { // If the intake motor is moving, stop it
                    intakeMotor.setVoltage(0);
                }


                if (noteAtPreIntakeSensor || noteAtIntakeSensor) { // If there is a note at the intake, start intaking and make sure that the timers are reset and stopped
                    currentState = IntakeState.INTAKING;
                    notePassedShooterSensor = false;
                    noteHitShooter = false;
                    timer.stop();
                    timer.reset();
                }

                break;
            case INTAKING:

                if (noteAtPreIntakeSensor || noteAtIntakeSensor || noteAtShooterSensor) { // If there is a note in the intake subsystem, make sure the motor is moving
                    if (intakeMotor.get() == 0) { // If the motor is not moving, make it move

                        intakeMotor.setVoltage(2.4);
                    }
                } else { // The note is not touching any of the sensors
                    if (!notePassedShooterSensor) { // If the note has not gone past the last sensor (meaning we haven't actually picked the note up)
                        currentState = IntakeState.IDLE;
                        intakeMotor.stopMotor();
                    }
                }

                if (noteAtShooterSensor) { // Now we know when the note hits the shooter sensor
                    if (!notePassedShooterSensor) {
                        notePassedShooterSensor = true;
                        timer.start();
                    }
                }

                if (notePassedShooterSensor && !noteAtShooterSensor && !noteHitShooter) { // TODO: tune this value. This timer starts when the note hits the last sensor, and this value will make the indexer go backwards for a short period of time to make sure the note is in the right spot.
                    noteHitShooter = true;
                    intakeMotor.setVoltage(-1.2);
                    timer.stop();
                    timer.reset();
                    timer.start();
                }

                if (timer.hasElapsed(0.1) && noteHitShooter) { // TODO: tune this value. This timer starts when the note hits the shooter wheels, and this value stops the intaking process when the timer reaches this value
                    currentState = IntakeState.HOLDING;
                    intakeMotor.stopMotor();
                    timer.stop();
                    timer.reset();
                }

                break;
            case HOLDING:
                intakeMotor.stopMotor();
                noteReady = true;

                break;
            case SHOOTING:

                if (timer.get() == 0) { // If the timer has not started, start the timer and the motor
                    intakeMotor.setVoltage(2.4);

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
    }
}
