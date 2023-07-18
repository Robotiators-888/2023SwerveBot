package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class SUB_Manuiplator extends SubsystemBase {
    private final CANSparkMax Extend = new CANSparkMax(Constants.Manuiplator.kMANUIP_EXTEND_MOTOR_CANID, MotorType.kBrushless);
    private final CANSparkMax Rotate = new CANSparkMax(Constants.Manuiplator.kMANUIP_ROTATE_MOTOR_CANID, MotorType.kBrushless);
    private final TalonSRX intakeMotor = new TalonSRX(Constants.Manuiplator.kMANUIP_INTAKE_MOTOR_CANID);    
    
    

    public void rotateArm(double speed){

        Rotate.set(speed);
    }


    
    private TrapezoidProfile m_profile;
    private Timer m_timer;

    //declare encoders
    private SparkMaxPIDController m_controller;
    private double m_setpoint;
    private RelativeEncoder m_encoder;
    private TrapezoidProfile.State targetState;
    private double feedforward;
    private double manualValue;

    public SUB_Manuiplator(){
        Rotate.restoreFactoryDefaults();
        Rotate.setOpenLoopRampRate(0.6); // motor takes 0.6 secs to reach desired power
        Rotate.setInverted(false);
        Rotate.setIdleMode(IdleMode.kBrake);
        m_encoder = Rotate.getEncoder();

    }
    
    public void periodic(){

   }

   public void driveMotor(int motor, double speed){
        switch(motor){
            case 1: Rotate.set(speed); break;
            case 2: Extend.set(speed); break;
            case 3: intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
        }
   }


   public void setTargetPosition(double setpoint, SUB_Manuiplator manip){
        if (setpoint != m_setpoint){
            m_setpoint = setpoint;
            updateMotionProfile();
        }
   }

   private void updateMotionProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.Manuiplator.kPivotConstraint, goal, state);
    m_timer.reset();
  }

  public void runAutomatic() {
    double elapsedTime = m_timer.get();
    if (m_profile.isFinished(elapsedTime)) {
      targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    }
    else {
      targetState = m_profile.calculate(elapsedTime);
    }

    //feedforward = Constants.Arm.kArmFeedforward.calculate(m_encoder.getPosition()+Constants.Arm.kArmZeroCosineOffset, targetState.velocity);
    m_controller.setReference(targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
  }
  public void runManual(double _power) {
    //reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and passively
    m_setpoint = m_encoder.getPosition();
    targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.Manuiplator.kPivotConstraint, targetState, targetState);
    //update the feedforward variable with the newly zero target velocity
    //feedforward = Constants.Manuiplator.kArmFeedforward.calculate(m_encoder.getPosition()+Constants.Manuiplator.kArmZeroCosineOffset, targetState.velocity);
    Rotate.set(_power + (feedforward / 12.0));
    manualValue = _power;
  }


}
