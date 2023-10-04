// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;

public class SUB_Limelight extends SubsystemBase {
  private NetworkTable table;

  /** Creates a new SUB_Limelight. */
  public SUB_Limelight() {
    this.table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  /**
   * Returns robot pose in terms of x,y,z position on the field in meters, and roll, pitch yaw, in degrees. 
   * @implNote (X, Y, Z (meters), ROLL, PITCH, YAW (degrees))
   * @return Botpose network tables entry in Pose3d
   */
  public Pose3d getPose(){
    NetworkTableEntry entry = table.getEntry("botpose");
    double[] poseArr = entry.getDoubleArray(new double[0]);

    // If the pose array is empty/contains the default array (double[0])
    if (poseArr.length == 0){
      return null;
    }

    // x,y and -yaw.
    return new Pose3d(new Pose2d(poseArr[0], poseArr[1], new Rotation2d(-poseArr[5])));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getTv() {
    if (table.getEntry("tv").getDouble(0) == 1) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Crosshair offset to target x-value
   * 
   * @return double of offset of target x-value
   */
  public double getTx() {
    return table.getEntry("tx").getDouble(0);
  }

  /**
   * Crosshair offset to target y-value
   * 
   * @return double of offset of target y-value
   */
  public double getTy() {
    return table.getEntry("ty").getDouble(0);
  }

  /**
   * Returns target pose in terms of x,y,z position on the field in meters, and roll, pitch yaw, in degrees. 
   * @implNote We assume that the array is shaped (X, Y, Z (meters), ROLL, PITCH, YAW (degrees)), however, ENSURE that this is the case.
   * @return Get the 3D transform needed to get to the target in Pose3D
   */
  public Transform3d getTargetTransform(){
    NetworkTableEntry entry = table.getEntry("target-pose_cameraspace");
    double[] resultArr = entry.getDoubleArray(new double[0]);

    // If the result array is empty/contains the default array (double[0])
    if (resultArr.length == 0){
      return null;
    }

    Translation3d transl3d = new Translation3d(resultArr[0], resultArr[1], resultArr[2]);
    Rotation3d rot3d = new Rotation3d(resultArr[3], resultArr[4], resultArr[5]);
    // x,y and -yaw.

    return 
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
