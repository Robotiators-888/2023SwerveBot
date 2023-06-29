package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Manuiplator extends SubsystemBase {
    private final CANSparkMax Extend = new CANSparkMax(Constants.Manuiplator.kMANUIP_EXTEND_MOTOR_CANID, MotorType.kBrushless);
    private final CANSparkMax Rotate = new CANSparkMax(Constants.Manuiplator.kMANUIP_ROTATE_MOTOR_CANID, MotorType.kBrushless);
    private final CANSparkMax Intake = new CANSparkMax(Constants.Manuiplator.kMANUIP_INTAKE_MOTOR_CANID, MotorType.kBrushless);
    
    

    }
