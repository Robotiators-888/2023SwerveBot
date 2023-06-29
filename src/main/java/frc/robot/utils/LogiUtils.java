package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogiUtils extends Joystick {
  
    public enum axis {
        kLEFTX(0),
        kLEFTY(1),
        kLEFTTRIIGER(2),
        kRIGHTTRIGGER(3),
        kRIGHTX(4),
        kRIGHTY(5);
        
        public final int value;

        axis(int value) {
          this.value = value;
        }
    }
    
    public enum button {
        kABUTTON(1),
        kBBUTTON(2),
        kXBUTTON(3),
        kYBUTTON(4),
        kLEFTBUMPER(5),
        kRIGHTBUMPER(6),
        kBACKBUTTON(7),
        kSTARTBUTTON(8),
        kLEFTJOYSTICKPRESS(9),
        kRIGHTJOYSTICKPRESS(10);
        
        public final int value;

        button(int value) {
            this.value = value;
        }
    }

    public LogiUtils(int port){
        super(port);
    }
    public boolean getAButtonPressed(){
        return getRawButton(button.kABUTTON.value);
    }       
    public boolean getBButtonPressed(){
        return getRawButton(button.kBBUTTON.value);
    } 
    public boolean getXButtonPressed(){
        return getRawButton(button.kXBUTTON.value);
    }             
    public boolean getYButtonPressed(){
        return getRawButton(button.kYBUTTON.value);
    }       
    public boolean getLeftBumperButtonPressed(){
        return getRawButton(button.kLEFTBUMPER.value);
    }   
    public boolean getRightBumperButtonPressed(){
        return getRawButton(button.kRIGHTBUMPER.value);
    }           
    public boolean getStartButtonPressed(){
        return getRawButton(button.kSTARTBUTTON.value);
    }       
    public boolean getBackButtonPressed(){
        return getRawButton(button.kBACKBUTTON.value);
    }       
    public boolean getLeftJoystickButtonPressed(){
        return getRawButton(button.kLEFTJOYSTICKPRESS.value);
    }       
    public boolean getRightJoystickButtonPressed(){
        return getRawButton(button.kRIGHTJOYSTICKPRESS.value);
    }  
    public double getLeftXAxis(){
        return getRawAxis(axis.kLEFTX.value);
    }
    public double getLeftYAxis(){
        return getRawAxis(axis.kLEFTY.value);
    }
    public double getRightXAxis(){
        return getRawAxis(axis.kRIGHTX.value);
    }
    public double getRightYAxis(){
        return getRawAxis(axis.kRIGHTY.value);
    }
    public double getLeftTriggerAxis(){
        return getRawAxis(axis.kLEFTTRIIGER.value);
    }
    public double getRightTriggerAxis(){
        return getRawAxis(axis.kRIGHTTRIGGER.value);
    }
}
    
