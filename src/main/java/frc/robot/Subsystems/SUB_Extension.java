// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Extension extends SubsystemBase {
    private final static CANSparkMax extendMotor = new CANSparkMax(Constants.Manuiplator.kMANUIP_EXTEND_MOTOR_CANID, MotorType.kBrushless);
    private RelativeEncoder extensionEncoder;

  /** Creates a new SUB_Extension. */
  public SUB_Extension() {
    extendMotor.restoreFactoryDefaults();
    extensionEncoder = extendMotor.getEncoder();
    extensionEncoder.setPosition(0);
    extendMotor.setIdleMode(IdleMode.kBrake);
    extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float) 85);
    extendMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) 0.1);
    extendMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    extendMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    extendMotor.burnFlash();
  }

  public static void driveMotor(double speed){
    extendMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extend", extensionEncoder.getPosition());
  }

  public static void extendStop() {
    extendMotor.set(0.0);
  }
}
