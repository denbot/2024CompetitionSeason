package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer; 
import frc.robot.subsystems.Intake; 


public class Lights extends SubsystemBase {
    private AddressableLED ledString; 
    private AddressableLEDBuffer ledBuffer; 
    private int amountOfLights; 
    private final Intake intake; 

    public Lights(Intake intakeSubsystem) {
        ledString = new AddressableLED(1); 
        this.amountOfLights = 24; 
        ledBuffer = new AddressableLEDBuffer(this.amountOfLights); 
        
        this.intake = intakeSubsystem; 
        ledString.setLength(this.amountOfLights);
        this.update();  
        ledString.start(); 

    }

    public void update() { 
        ledString.setData(ledBuffer); 
    }

    public void solid (int start, int finish, int hue, int sat, int val) {
        for (var i = start; i < finish; i++) {
            ledBuffer.setHSV(i, hue, sat, val); 
        
        
        }    
        update(); 

    }

    public void sensorCheck(){

        if (intake.noteAtShooterSensor()) {
            solid(0, 24, 60, 255, 255); 
        } else { 
            solid(0, 24, 0,0,0); 
        }

    }

    public void alignCheck(){}
}
