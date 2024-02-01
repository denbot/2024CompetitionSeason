// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
 
public class Intake extends SubsystemBase{
    private final DigitalInput input0 = new DigitalInput(0);
    private final DigitalInput input1 = new DigitalInput(1);
    private final DigitalInput input2 = new DigitalInput(2);

    private final TalonFX intakeMotor = new TalonFX(0);
    /** Creates a wait function that can be called on using wait() */
    public static void wait(int ms) {
        try
        {
            //Uses milliseconds as sleep time
            Thread.sleep(ms);
        }
        catch(InterruptedException ex){
            Thread.currentThread().interrupt();
        }
    }
    boolean inside = false;
    boolean readyToShoot = false;
    boolean failSafe = false;
    boolean status = intakeMotor.isAlive();
    double intakeMotorSpeed = 0;
    /** Creates a new intake. */
    public Intake(){
        //SmartDashboard displays to test sensors
        SmartDashboard.putBoolean("Motor Functional?", status);
        SmartDashboard.putBoolean("Engage Intake",input0.get());
        SmartDashboard.putBoolean("Intaking", input1.get());
        SmartDashboard.putBoolean("Stored?", input2.get());
        SmartDashboard.putBoolean("Ready To Fire", readyToShoot);
        //If there is no note inside and a note is detected we start the intake motors
        if (input0.get() == true && inside == false){
            //Starting Intake Motors
            // intakeMotor.set(0);
        }
        if (input1.get() == true && inside == false){
            //Telling our code that we have a note intaking and we cannot pick up another note
            inside = true;
            failSafe = true;
            
        }
        //Failsafe to prevent incorrect sensor detections
        if (failSafe == true && inside == true){
            //Time is subject to change
            wait(5000);
            if (input2.get() == false){
                inside = false;
                failSafe = false;
            }
            else return;
        }
        if (input2.get() == true && readyToShoot == false){
            //Detects note inside intake and stops intake motors
            readyToShoot = true;
            //intakeMotor.set(0);
            //wait(0);
            //intakeMotor.set(0);
            failSafe = false;
        } else if(input2.get() == false && readyToShoot == true){
            //Note has been fired
            readyToShoot = false;
            inside = false;
            //intakeMotor.set(0);
        }

    }
}