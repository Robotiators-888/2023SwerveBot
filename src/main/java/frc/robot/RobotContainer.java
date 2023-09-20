// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.SUB_Drivetrain;
import frc.robot.Subsystems.SUB_Extension;
import frc.robot.Subsystems.SUB_Intake;
import frc.robot.Subsystems.SUB_Manuiplator;
import frc.robot.utils.AutoBuilder;
import frc.robot.utils.LogiUtils;

public class RobotContainer {
  public static SUB_Drivetrain drivetrain = new SUB_Drivetrain();
  public static SUB_Manuiplator manuiplator = new SUB_Manuiplator();
  public static SUB_Intake intake = new SUB_Intake();
  public static SUB_Extension extension = new SUB_Extension();
  public static RobotManager manager = RobotManager.getInstance();

  Joystick joystick = new Joystick(0);
  LogiUtils DriverC = new LogiUtils(0);
  LogiUtils logiUtils1 = new LogiUtils(1);
  JoystickButton leftBumperC = DriverC.getLeftBumperButtonPressed();
  JoystickButton RightBumperC = DriverC.getRightBumperButtonPressed();
  JoystickButton leftBumper = logiUtils1.getLeftBumperButtonPressed();
  JoystickButton rightBumper = logiUtils1.getRightBumperButtonPressed();
  
  JoystickButton aButton = logiUtils1.getAButtonPressed(); //Ground
  JoystickButton yButton = logiUtils1.getYButtonPressed(); //Stow/up
  JoystickButton xButton = logiUtils1.getXButtonPressed(); //Single Feed
  JoystickButton bButton = logiUtils1.getBButtonPressed(); //Scoring Height

  private AutoBuilder autoBuilder = new AutoBuilder(drivetrain);

  public RobotContainer() {
    configureBindings();

    // Configure default commands

    
    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> drivetrain.drive(
                -MathUtil.applyDeadband(DriverC.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(DriverC.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(DriverC.getRawAxis(4), OIConstants.kDriveDeadband),
                true, true),
                drivetrain));
  }

  public void periodic(){
    SmartDashboard.putNumber("Scoring height", manager.getScoringHeight());
  }

    // Flight Controller
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
            () ->
                drivetrain.drive(
                    -MathUtil.applyDeadband(joystick.getRawAxis(1), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(joystick.getRawAxis(0), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(joystick.getRawAxis(4), OIConstants.kDriveDeadband),
                    true,
                    true),
            drivetrain));

  private void configureBindings() {
    aButton.onTrue(new InstantCommand(()->manuiplator.setTargetPosition(Constants.Manuiplator.kGroundPosition, manuiplator)));
    bButton.onTrue(new InstantCommand(()->manuiplator.setTargetPosition(manager.getScoringHeight(), manuiplator)));
    yButton.onTrue(new InstantCommand(()->manuiplator.setTargetPosition(Constants.Manuiplator.kStow, manuiplator)));
    xButton.onTrue(new InstantCommand(()->manuiplator.setTargetPosition(Constants.Manuiplator.kSingleFeeder, manuiplator)));

    leftBumperC.whileTrue(new RunCommand(()->SUB_Intake.intakeIn(),intake)).onFalse(new InstantCommand(()->SUB_Intake.intakeStop()));
    RightBumperC.whileTrue(new RunCommand(()->SUB_Intake.intakeOut(),intake)).onFalse(new InstantCommand(()->SUB_Intake.intakeStop()));

    manuiplator.setDefaultCommand(new RunCommand(()-> manuiplator.runAutomatic(), manuiplator));

    leftBumper.whileTrue(new RunCommand(()->SUB_Extension.driveMotor(Constants.Extension.kReverseSpeed), extension)).onFalse(new InstantCommand(()->SUB_Extension.extendStop()));
    rightBumper.whileTrue(new RunCommand(()->SUB_Extension.driveMotor(Constants.Extension.kForwardSpeed), extension)).onFalse(new InstantCommand(()->SUB_Extension.extendStop()));
    left.whileTrue(new RunCommand(()->SUB_Extension.driveMotor(Constants.Extension.kReverseSpeed), extension)).onFalse(new InstantCommand(()->SUB_Extension.extendStop()));
    right.whileTrue(new RunCommand(()->SUB_Extension.driveMotor(Constants.Extension.kReverseSpeed), extension)).onFalse(new InstantCommand(()->SUB_Extension.extendStop()));


  }

  public Command getAutonmousCommand(){
    return autoBuilder.getSelectedAuto();
  }

  
  
}
