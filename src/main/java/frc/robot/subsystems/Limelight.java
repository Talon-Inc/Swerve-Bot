// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv"); //Whether the limelight has any valid targets (0 or 1)
  NetworkTableEntry tx = table.getEntry("tx"); //Horizontal Offset From Crosshair To Target
  NetworkTableEntry ty = table.getEntry("ty"); //Vertical Offset From Crosshair To Target
  NetworkTableEntry ta = table.getEntry("ta"); //Target Area
  NetworkTableEntry tid = table.getEntry("tid"); //ID of the primary in-view AprilTag

  // Reads values periodically
  double v = tv.getDouble(0);
  double x = tx.getDouble(0);
  double y = ty.getDouble(0);
  double a = ta.getDouble(0);
  double[] id = tid.getDoubleArray(new double[6]);

  double angleToGoalRadians = y * (3.14159/180); //vertical angle from camera to April Tag in Radians


  /** Creates a new Limelight. */
  public Limelight() {
    //post to smart dashboard periodically
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightV", v);
    SmartDashboard.putNumber("LimelightA", a);
    SmartDashboard.putNumberArray("LimelightID", id);
  }

  public double estimateDistance() {
    double distanceFromLimelightToGoal = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoal;
  }
}
