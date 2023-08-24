package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class SUB_Manuiplator extends SubsystemBase {
    //private final CANSparkMax Extend = new CANSparkMax(Constants.Manuiplator.kMANUIP_EXTEND_MOTOR_CANID, MotorType.kBrushless);
    private final CANSparkMax rotateMotor = new CANSparkMax(Constants.Manuiplator.kMANUIP_ROTATE_MOTOR_CANID, MotorType.kBrushless);
    private final TalonSRX intakeMotor = new TalonSRX(Constants.Manuiplator.kMANUIP_INTAKE_MOTOR_CANID);    
    public final SparkMaxAbsoluteEncoder rotateEncoder = rotateMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private Timer m_timer;

    public void rotateArm(double speed){

        rotateMotor.set(speed);
        
    }


    
    private TrapezoidProfile m_profile;
    

    //declare encoders
    private SparkMaxPIDController m_controller;
    private double m_setpoint;
    private AbsoluteEncoder m_encoder;
    private TrapezoidProfile.State targetState;
    private double feedforward;
    private double manualValue;

    double setpoint = Constants.Manuiplator.kGroundPosition;
    //Counteract Gravity on Arm, Currently lbsArm is arbitrary (For kG of FF)
    double lbsArm = 30.0;
    double gravitional_force_in_Kg = (lbsArm*4.44822162)/9.8;

    public SUB_Manuiplator(){
        rotateMotor.setOpenLoopRampRate(0.6); // motor takes 0.6 secs to reach desired power
        rotateMotor.setInverted(false);
        rotateMotor.setIdleMode(IdleMode.kBrake);
        m_encoder = rotateMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_controller = rotateMotor.getPIDController();
        m_timer = new Timer();
        m_timer.start();
        m_timer.reset(); 
        setLimits();
    }

        
    public void periodic(){
      SmartDashboard.putNumber("setpoint", setpoint);
      SmartDashboard.putNumber("VelocityEncoder", m_encoder.getVelocity());
      SmartDashboard.putNumber("PositionEncoder", m_encoder.getPosition());
   }

   public void driveMotor(int motor, double speed){
        switch(motor){
            case 1: rotateMotor.set(speed); break;
            //case 2: Extend.set(speed); break;
            case 3: intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
        }
   }

   public void setLimits(){
    //set soft limits and current limits for how far the manip can move
    rotateMotor.setSmartCurrentLimit(40, 20);}
    
  //   rotateMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
  //   rotateMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

  //   // stops motor at 130 encoder clicks, (touching the ground)
  //   rotateMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 2048);
  //   // stops motor at 0 encoder clicks when reversing, (touching the robot)
  //   rotateMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 8);
  //  }
   public void setTargetPosition(double setpoint, SUB_Manuiplator manip){
        if (setpoint != m_setpoint){
            m_setpoint = setpoint;
            updateMotionProfile();
        }
   }

   private void updateMotionProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(m_encoder.getPosition()
    , m_encoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.Manuiplator.kArmMotionConstraint, goal, state);
    m_timer.reset();
  }

  public void armMoveVoltage(double volts) {
    //towerMotor.set(pid.calculate(getRotations(), setpoint) + feedforward.calculate(Constants.FF_Velocity, Constants.FF_Accel));
    rotateMotor.setVoltage(volts+getAutoBalanceVolts());// sets voltage of arm -12 to 12 volts
}

// calculates volts to counteract gravity based on position of the arm
public double getAutoBalanceVolts(){
    // Math.cos(theta) as more downward force increases near 0,180 degrees
   return (Constants.Manuiplator.FF_kG*Math.cos(Math.toRadians(calculateDegreesRotation())));
}

// converts encoder clicks of arm into arms rotation in degrees
public double calculateDegreesRotation(){
    double encoderClicksToDegrees = 8192.00/360.00;
    return (encoderClicksToDegrees*getRotations());
}

// get arm encoder clicks
public double getRotations(){
  
    //gets position
    return m_encoder.getPosition();
}
  public void runAutomatic() {
    double elapsedTime = m_timer.get();
    if (m_profile.isFinished(elapsedTime)) {
      targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    }
    else {
      targetState = m_profile.calculate(elapsedTime);
    }

    feedforward = Constants.Manuiplator.kArmFeedforward.calculate(m_encoder.getPosition(), targetState.velocity);
    m_controller.setReference(targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
  }
  public void runManual(double _power) {
    //reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and passively
    m_setpoint = m_encoder.getPosition();
    targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
   // m_profile = new TrapezoidProfile(Constants.PIDConstants.kPivotConstraint, targetState, targetState);
    //update the feedforward variable with the newly zero target velocity
    //feedforward = Constants.Manuiplator.kArmFeedforward.calculate(m_encoder.getPosition()+Constants.Manuiplator.kArmZeroCosineOffset, targetState.velocity);
    rotateMotor.set(_power + (feedforward / 12.0));
    manualValue = _power;
  }


}
