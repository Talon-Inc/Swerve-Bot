// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AprilTagAiming extends CommandBase {

  DriveSubsystem swerveDrive;
  Limelight limelight;

  PIDController pidController;

  double Kp = -0.0078;

  /** Creates a new AprilTagAiming. */
  public AprilTagAiming(DriveSubsystem swerveDrive, Limelight limelight) {

    this.swerveDrive = swerveDrive;
    this.limelight = limelight;

    // pidController = new PIDController(0.001, .09, 0);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.turnOnLED();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double headingError = limelight.getXCrossHair();
    SmartDashboard.putNumber("Heading Error", headingError);

    double rot = Kp * tx;

    /* If the limelight overshoots the target to a point it cannot read it, it kept on spinning.
    So, this conditional statement will make it stop spinning if it overshoots it and can't detect
    the April Tag anymore
     */ 
    if (limelight.getIsDetecting() == false) {
      rot = 0;
    }
    // double rotSpeed = pidController.calculate(headingError, 0);
    swerveDrive.drive(0, 0, rot, isFinished(), isFinished());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.turnOffLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}