// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.SUB_Drivetrain;
import frc.robot.Subsystems.SUB_Manuiplator;
import frc.robot.utils.AutoBuilder;

public class RobotContainer{

  public static SUB_Drivetrain drivetrain = new SUB_Drivetrain();
  public static SUB_Manuiplator manuiplator = new SUB_Manuiplator();

  Joystick DriverC = new Joystick(0);
  Joystick OperatorC = new Joystick(1);

  private JoystickButton aButton = new JoystickButton(OperatorC, 1);
  private JoystickButton bButton = new JoystickButton(OperatorC, 2);
  private JoystickButton yButton = new JoystickButton(OperatorC, 4);
  private JoystickButton xButton = new JoystickButton(OperatorC, 3);
  private JoystickButton rBumper = new JoystickButton(DriverC, 5);
  private JoystickButton lBumper = new JoystickButton(DriverC, 6);
  private JoystickButton backButton = new JoystickButton(OperatorC, 7);

  private AutoBuilder autoBuilder = new AutoBuilder(drivetrain);

  public RobotContainer() {
    configureBindings();

    // Configure default commands

    //Flight Controller
    // drivetrain.setDefaultCommand(
    //     new RunCommand(
    //         () -> drivetrain.drive(
    //             -MathUtil.applyDeadband(joystick.getRawAxis(1), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(joystick.getRawAxis(0), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(joystick.getRawAxis(2), OIConstants.kDriveDeadband),
    //             true, true),
    //             drivetrain));

    // Logi Controller
    
    
    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> drivetrain.drive(
                -MathUtil.applyDeadband(DriverC.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(DriverC.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(DriverC.getRawAxis(4), OIConstants.kDriveDeadband),
                true, true),
                drivetrain));
  }

  private void configureBindings() {
    new Trigger(
      ()->(Math.abs(Math.pow(DriverC.getRawAxis(2),2)-Math.pow(DriverC.getRawAxis(3), 2)) > 0.1))
      .whileTrue(
        new RunCommand(()->manuiplator.driveMotor(1, Math.pow(DriverC.getRawAxis(2),2)-Math.pow(DriverC.getRawAxis(3), 2)), 
        manuiplator));
    
    rBumper.onTrue(new RunCommand(()->manuiplator.driveMotor(3, Constants.Manuiplator.INTAKE_CONE_SPEED), manuiplator));
    
    
  }

  public Command getAutonomousCommand() {
    return autoBuilder.getSelectedAuto();
  }
}
