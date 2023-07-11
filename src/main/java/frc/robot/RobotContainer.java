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
import frc.robot.utils.AutoBuilder;

public class RobotContainer{

  public static SUB_Drivetrain drivetrain = new SUB_Drivetrain();

  Joystick joystick = new Joystick(0);

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
                -MathUtil.applyDeadband(joystick.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(joystick.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(joystick.getRawAxis(4), OIConstants.kDriveDeadband),
                true, true),
                drivetrain));

    // test.setDefaultCommand(
    //     new RunCommand(
    //         () -> test.drive(
    //             -MathUtil.applyDeadband(joystick.getRawAxis(1), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(joystick.getRawAxis(0), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(joystick.getRawAxis(2), OIConstants.kDriveDeadband),
    //             true, true),
    //             test));



  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoBuilder.getSelectedAuto();
  }
}
