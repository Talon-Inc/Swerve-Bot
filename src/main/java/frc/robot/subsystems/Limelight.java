// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv"); //Whether the limelight has any valid targets (0 if no target; 1 if target)
  NetworkTableEntry tx = table.getEntry("tx"); //Horizontal Offset From Crosshair To Target
  NetworkTableEntry ty = table.getEntry("ty"); //Vertical Offset From Crosshair To Target
  NetworkTableEntry ta = table.getEntry("ta"); //Target Area
  NetworkTableEntry tid = table.getEntry("tid"); //ID of the primary in-view AprilTag
  NetworkTableEntry led = table.getEntry("ledMode");

  NetworkTableEntry robotPoseTargetSpace = table.getEntry("botpose_targetspace");
  NetworkTableEntry cameraPoseTargetSpace = table.getEntry("camerapose_targetspace");

  // Reads values periodically
  double v = tv.getDouble(0);
  double x = tx.getDouble(0);
  double y = ty.getDouble(0);
  double a = ta.getDouble(0);
  double[] id = tid.getDoubleArray(new double[6]);

  double angleToGoalRadians = y * (3.14159/180); //vertical angle from camera to April Tag in Radians

  double distanceToTarget = 0;
  double xToTarget, yToTarget, rotToTarget;

  float kP = -.1f;

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

    double[] botPositionArray = robotPoseTargetSpace.getDoubleArray(new double[6]);
    double[] camPositionArray = cameraPoseTargetSpace.getDoubleArray(new double[6]);

    xToTarget = botPositionArray[0];
    yToTarget = botPositionArray[2]; //This is distance in z-axis of limelight
    rotToTarget = camPositionArray[5];

    distanceToTarget = Math.hypot(xToTarget, yToTarget); //Distance from robot to April Tag

  }

  // public double estimateDistance() {
  //   double distanceFromLimelightToGoal = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  //   return distanceFromLimelightToGoal;
  // }

  public double[] getBotPositionToTargetSpace() {
    double[] botPositionArray = robotPoseTargetSpace.getDoubleArray(new double[6]);
    return botPositionArray;
  }

  public void turnOnLED() {
    led.setNumber(3);
  }

  public void turnOffLED() {
    led.setNumber(1);
  }

  public double isDetecting() {
    return v;
  }

  public double getDistance() {
    return distanceToTarget;
  }

  public double getXCrossHair() {
    return x;
  }

  public double getXDistance() {
    return xToTarget;
  }

  public double getYDistance() {
    return yToTarget;
  }

  public Rotation2d getRotationToTargetPlane() {
    return Rotation2d.fromDegrees(rotToTarget);
  }

  public boolean getIsDetecting() {
    int id = (int) tid.getInteger(Integer.MAX_VALUE);
    boolean isDetecting = tid.getInteger(0) == 1 || id >= 1 || id <= 8;
    return isDetecting;
  }
}
