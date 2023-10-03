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

  // find h1, h2, a1 of current limelight position
  
  // public double getDistance() {
  //   double h1 = 35.125;
  //   double h2 = 111;
  //   double a1 = 0.4625123;
  //   double a2 = Math.toRadians(this.getTy());

  //   return (double) ((h2 - h1) / (Math.tan(a1 + a2)));
  // }
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
