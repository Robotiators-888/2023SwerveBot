// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final static TalonSRX intakeMotor = new TalonSRX(Constants.Manuiplator.kMANUIP_INTAKE_MOTOR_CANID);

  public static void intakeIn(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -0.75);
  }
  public static void intakeOut(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput,0.75);
  }
  public static void intakeStop(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput,0);
  }
  
  public void runIntake(double speed){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public SUB_Intake() {
    intakeMotor.enableCurrentLimit(true);
    intakeMotor.configPeakCurrentLimit(50);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
