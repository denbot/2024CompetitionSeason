package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter; 


public class Lights extends SubsystemBase {
    private AddressableLED ledString; 
    private AddressableLEDBuffer ledBuffer; 
    private int amountOfLights; 
    private final Shooter shooter; 

    public Lights(Shooter shooterSubsystem ) {
        ledString = new AddressableLED(2);
        this.amountOfLights = 24; 
        ledBuffer = new AddressableLEDBuffer(this.amountOfLights); 
        
        this.shooter = shooterSubsystem; 
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

        if (shooter.isNoteInShooter()) {
            solid(0, 7, 60, 0, 255); 
            solid(17, 24, 60, 0, 255);
        } else { 
            solid(0, 7, 0, 0, 0); 
            solid(17, 24, 0, 0, 0);
        }

    }

    public void alignCheck(){
        if (LimelightHelpers.getTX("")!=0.0) {


            if (Math.abs(LimelightHelpers.getTX(""))<5) {
                solid(8,16,60,255,255); 

            }else {
                 solid(8,16,0,255,255); 
            }

       

        } else {
            solid(8, 16, 120, 255, 255);
        }
    }
}
