// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SUB_Limelight extends SubsystemBase {
  private NetworkTable table;

  /** Creates a new SUB_Limelight. */
  public SUB_Limelight() {
    this.table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  /**
   * Returns robot pose in terms of x,y,z position on the field in meters, and roll, pitch yaw, in degrees. 
   * @return Botpose network tables entry (X, Y, Z (meters), ROLL, PITCH, YAW (degrees))
   */
  public Pose2d getPose(){
    NetworkTableEntry entry = table.getEntry("botpose");
    double[] poseArr = entry.getDoubleArray(new double[0]);

    if (poseArr.length > 0){
      return null;
    }
    // x,y and -yaw.
    return new Pose2d(poseArr[0], poseArr[1], new Rotation2d(-poseArr[5]));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
