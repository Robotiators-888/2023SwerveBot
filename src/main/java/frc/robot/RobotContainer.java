// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.SUB_Drivetrain;
import frc.robot.Subsystems.SUB_Manuiplator;
import frc.robot.utils.AutoBuilder;
 
import frc.robot.utils.LogiUtils;

public class RobotContainer {
  

  public static SUB_Drivetrain drivetrain = new SUB_Drivetrain();
  SUB_Manuiplator manuiplator = new SUB_Manuiplator();
  //SUB_SwerveModuleTest test = new SUB_SwerveModuleTest();
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
  JoystickButton bButton = logiUtils1.getBButtonPressed(); // High, colin is high

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

  private void configureBindings() {
    // new Trigger(
    //   ()->(Math.abs(Math.pow(DriverC.getRawAxis(2),2)-Math.pow(DriverC.getRawAxis(3), 2)) > 0.1))
    //   .whileTrue(
    //     new RunCommand(()->manuiplator.driveMotor(1, Math.pow(DriverC.getRawAxis(2),2)-Math.pow(DriverC.getRawAxis(3), 2)), 
    //    manuiplator));
    
        
    
    
    //     rightBumper.onTrue(new RunCommand(()->manuiplator.driveMotor(3, Constants.Manuiplator.INTAKE_CONE_SPEED), manuiplator));
    

    aButton.onTrue(new InstantCommand(()->manuiplator.setTargetPosition(Constants.Manuiplator.kGroundPosition, manuiplator)));
    bButton.onTrue(new InstantCommand(()->manuiplator.setTargetPosition(Constants.Manuiplator.kScoreHigh, manuiplator)));
    yButton.onTrue(new InstantCommand(()->manuiplator.setTargetPosition(Constants.Manuiplator.kStow, manuiplator)));
    xButton.onTrue(new InstantCommand(()->manuiplator.setTargetPosition(Constants.Manuiplator.kSingleFeeder, manuiplator)));

    // leftBumper.onTrue(new InstantCommand(()->manuiplator.intakeIn(),manuiplator));
    // rightBumper.onTrue(new InstantCommand(()->manuiplator.intakeOut(),manuiplator));
    // new Trigger(()->(!leftBumper.getAsBoolean() && !rightBumper.getAsBoolean())).onFalse(new InstantCommand(()->manuiplator.intakeStop()));

    //manuiplator.setDefaultCommand(new RunCommand(()-> {manuiplator.armMoveVoltage(Constants.Manuiplator.kArmFeedforward);},manuiplator));
    manuiplator.setDefaultCommand(new RunCommand(()-> manuiplator.runAutomatic(), manuiplator));

    leftBumperC.whileTrue(new RunCommand(()->manuiplator.armMoveVoltage(-.6),manuiplator));
    RightBumperC.whileTrue(new RunCommand(()->manuiplator.armMoveVoltage(6),manuiplator));
    // new Trigger(()->(!leftBumper.getAsBoolean() && !rightBumper.getAsBoolean())).onFalse(new InstantCommand(()->manuiplator.armMoveVoltage(0)));

    // test.setDefaultCommand(
    //     new RunCommand(
    //         () -> test.drive(
    //             -MathUtil.applyDeadband(joystick.getRawAxis(1), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(joystick.getRawAxis(0), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(joystick.getRawAxis(2), OIConstants.kDriveDeadband),
    //             true, true),
    //             test));
    
    // leftBumper.whileTrue(new InstantCommand(()->manuiplator.rotateArm(.25)));
    // rightBumper.whileTrue(new InstantCommand(()->manuiplator.rotateArm(-.25)));
    // new Trigger(()->(!leftBumper.getAsBoolean() && !rightBumper.getAsBoolean())).onTrue(new InstantCommand(()->manuiplator.rotateArm(0)));


  }

  public Command getAutonmousCommand(){
    return autoBuilder.getSelectedAuto();
  }

  
  
}
