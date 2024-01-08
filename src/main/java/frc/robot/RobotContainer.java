// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.CMD_DriveToTarget;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.SUB_Drivetrain;
import frc.robot.Subsystems.SUB_Limelight;
import frc.robot.utils.AutoBuilder;
import frc.robot.utils.LogiUtils;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;

public class RobotContainer {
  public static SUB_Drivetrain drivetrain = new SUB_Drivetrain();
  public static SUB_Limelight limelight = new SUB_Limelight();
  public static RobotManager manager = RobotManager.getInstance();


  Joystick joystick = new Joystick(0);
  LogiUtils DriverC = new LogiUtils(0);
  LogiUtils logiUtils1 = new LogiUtils(1);
  JoystickButton leftBumperC = DriverC.getLeftBumperButtonPressed();
  JoystickButton RightBumperC = DriverC.getRightBumperButtonPressed();
  JoystickButton leftBumper = logiUtils1.getLeftBumperButtonPressed();
  JoystickButton rightBumper = logiUtils1.getRightBumperButtonPressed();
  JoystickButton aButton = logiUtils1.getAButtonPressed(); // Ground
  JoystickButton yButton = logiUtils1.getYButtonPressed(); // Stow/up
  JoystickButton xButton = logiUtils1.getXButtonPressed(); // Single Feed
  JoystickButton bButton = logiUtils1.getBButtonPressed(); // Scoring Height
  JoystickButton startButton = logiUtils1.getStartButtonPressed();
  JoystickButton backButton = logiUtils1.getBackButtonPressed();
  DataLog log;
  DoubleArrayLogEntry poseEntry;

  public RobotContainer() {
    configureBindings();

    // Configure default commands

    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> drivetrain.drive(
                -MathUtil.applyDeadband(Math.pow(DriverC.getRawAxis(1), 3), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.pow(DriverC.getRawAxis(0), 3), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(DriverC.getRawAxis(4), OIConstants.kDriveDeadband),
                true, true),
                drivetrain));
    
    log = DataLogManager.getLog();
    poseEntry = new DoubleArrayLogEntry(log, "odometry/pose");
  }


  private void configureBindings() {

    startButton.whileTrue(new CMD_DriveToTarget(limelight, drivetrain)).onFalse(new InstantCommand(()->drivetrain.drive(0,0,0,true,true)));
  }

  public Command getAutonmousCommand() {
    return new RunCommand(()->System.out.println("No auto is crazy"));
  }

  public void periodic(){
    Pose2d visionPose = limelight.getPose();
    if (visionPose != null){
      drivetrain.addVisionMeasurement(visionPose);
    }

    Pose2d current_pose = drivetrain.getPose();
    double x = current_pose.getX();
    double y = current_pose.getY();
    double rotation_degs = current_pose.getRotation().getDegrees();
    double[] loggedPose = {x, y, rotation_degs};
    poseEntry.append(loggedPose);
  }
}
