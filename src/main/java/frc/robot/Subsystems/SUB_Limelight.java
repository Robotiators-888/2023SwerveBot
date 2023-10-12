// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.Limelight.*;

public class SUB_Limelight extends SubsystemBase {
  private NetworkTable table;

  /** Creates a new SUB_Limelight. */
  public SUB_Limelight() {
    this.table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
  }

  /**
   * Returns robot pose in terms of x,y,z position on the field in meters, and roll, pitch yaw, in degrees. 
   * @implNote (X, Y, Z (meters), ROLL, PITCH, YAW (degrees))
   * @return Botpose network tables entry in Pose2d
   */
  public Pose2d getPose(){
    double[] poseArr = LimelightHelpers.getBotPose(LIMELIGHT_NAME);

    // If the pose array is empty/contains the default array (double[0])
    if (poseArr.length == 0){
      return null;
    }

    // x,y and -yaw.
    return new Pose2d(poseArr[0], poseArr[1], new Rotation2d(-poseArr[5]));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ID", LimelightHelpers.getFiducialID(LIMELIGHT_NAME));
  }

  public boolean getTv() {
    return LimelightHelpers.getTV(LIMELIGHT_NAME);
  }

  /**
   * Crosshair offset to target x-value
   * 
   * @return double of offset of target x-value
   */
  public double getTx() {
    return LimelightHelpers.getTX(LIMELIGHT_NAME);
  }

  /**
   * Crosshair offset to target y-value
   * 
   * @return double of offset of target y-value
   */
  public double getTy() {
    return LimelightHelpers.getTY(LIMELIGHT_NAME);
  }

  /**
   * Returns the target's pose 2d relative to the robot (the robot is the origin)
   * @return Get the 3D pose needed of target
   */
  public Transform2d getTargetTransform(){
    return new Transform2d(new Pose2d(), LimelightHelpers.getTargetPose3d_RobotSpace(LIMELIGHT_NAME).toPose2d());
  }
 

  /**
   * Sets LED Mode
   */
  public void setLed(int value) {
    table.getEntry("ledMode").setNumber(value);

  }

  /**
   * Sets pipeline
   */
  public void setPipline(int value) {
    table.getEntry("pipeline").setNumber(value);

  }
}
