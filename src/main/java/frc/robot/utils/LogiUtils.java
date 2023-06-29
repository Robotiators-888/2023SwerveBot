package frc.robot.utils;
import edu.wpi.first.wpilibj.Joystick;

// To Import This Class Use: LogiUtils logiUtils = new LogiUtils("Your Controller USB Port #");
/**
 * Wrapper Class For Logitech F310 Controller
 */
public class LogiUtils extends Joystick {
  /**
   * List of Axies and Their Corresponding Numbers 
   */
    public enum axis {
        kLEFTX(0),
        kLEFTY(1),
        kLEFTTRIIGER(2),
        kRIGHTTRIGGER(3),
        kRIGHTX(4),
        kRIGHTY(5);
        
        public final int value;
        /**
         * The Axis # And Corresponding Value
         * @param value
         */
        axis(int value) {
          this.value = value;
        }
    }
    /**
     * List of Buttons and Their Corresponding Numbers 
     */
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
        /**
         * The Button # And Corresponding Value
         * @param value
         */
        button(int value) {
            this.value = value;
        }
    }
    /**
     * The Constructor That Allows The User To Pass a Port Number to This Class
     * @param port,
     */
    public LogiUtils(int port){
        super(port);
    }
    /**
     * 
     * @return A Boolean Value That Corresponds to The State of The Controler A Button
     */
    public boolean getAButtonPressed(){
        return getRawButton(button.kABUTTON.value);
    }       
    /**
     * 
     * @return A Boolean Value That Corresponds to The State of The Controler B Button
     */
    public boolean getBButtonPressed(){
        return getRawButton(button.kBBUTTON.value);
    } 
    /**
     * 
     * @return A Boolean Value That Corresponds to The State of The Controler X Button
     */
    public boolean getXButtonPressed(){
        return getRawButton(button.kXBUTTON.value);
    }     
    /**
     * 
     * @return A Boolean Value That Corresponds to The State of The Controler Y Button
     */        
    public boolean getYButtonPressed(){
        return getRawButton(button.kYBUTTON.value);
    }       
    /**
     * 
     * @return A Boolean Value That Corresponds to The State of The Controler Left Bumper Button
     */        
    public boolean getLeftBumperButtonPressed(){
        return getRawButton(button.kLEFTBUMPER.value);
    }   
    /**
     * 
     * @return A Boolean Value That Corresponds to The State of The Controler Right Bumper Button
     */   
    public boolean getRightBumperButtonPressed(){
        return getRawButton(button.kRIGHTBUMPER.value);
    }  
    /**
     * 
     * @return A Boolean Value That Corresponds to The State of The Controler Start Button On The Front of The Controller
     */            
    public boolean getStartButtonPressed(){
        return getRawButton(button.kSTARTBUTTON.value);
    }   
    /**
     * 
     * @return A Boolean Value That Corresponds to The State of The Controler Back Button On The Front of The Controller
     */      
    public boolean getBackButtonPressed(){
        return getRawButton(button.kBACKBUTTON.value);
    }      
    /**
     * 
     * @return A Boolean Value That Corresponds to The State of The Controler Left Joystick Press Sensor
     */   
    public boolean getLeftJoystickButtonPressed(){
        return getRawButton(button.kLEFTJOYSTICKPRESS.value);
    }       
    /**
     * 
     * @return A Boolean Value That Corresponds to The State of The Controler Right Joystick Press Sensor
     */   
    public boolean getRightJoystickButtonPressed(){
        return getRawButton(button.kRIGHTJOYSTICKPRESS.value);
    }  
    /**
     * 
     * @return A Double Value That Corresponds to The X Axis Value of The Left Controller Joystick
     */   
    public double getLeftXAxis(){
        return getRawAxis(axis.kLEFTX.value);
    }
    /**
     * 
     * @return A Double Value That Corresponds to The Y Axis Value of The Left Controller Joystick
     */  
    public double getLeftYAxis(){
        return getRawAxis(axis.kLEFTY.value);
    }
    /**
     * 
     * @return A Double Value That Corresponds to The X Axis Value of The Right Controller Joystick
     */  
    public double getRightXAxis(){
        return getRawAxis(axis.kRIGHTX.value);
    }
    /**
     * 
     * @return A Double Value That Corresponds to The Y Axis Value of The Right Controller Joystick
     */  
    public double getRightYAxis(){
        return getRawAxis(axis.kRIGHTY.value);
    }
    /**
     * 
     * @return A Double Value That Corresponds to The Axis Value of The Left Bumper
     */  
    public double getLeftTriggerAxis(){
        return getRawAxis(axis.kLEFTTRIIGER.value);
    }
    /**
     * 
     * @return A Double Value That Corresponds to The Axis Value of The Right Bumper
     */  
    public double getRightTriggerAxis(){
        return getRawAxis(axis.kRIGHTTRIGGER.value);
    }
}
    
