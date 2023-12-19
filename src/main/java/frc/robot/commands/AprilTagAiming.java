// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AprilTagAiming extends CommandBase {

  DriveSubsystem swerveDrive;
  Limelight limelight;

  double Kp = 0.3;

  /** Creates a new AprilTagAiming. */
  public AprilTagAiming(DriveSubsystem swerveDrive, Limelight limelight) {

    this.swerveDrive = swerveDrive;
    this.limelight = limelight;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double headingError = limelight.getXDistance();

    double rot = Kp * headingError;
    swerveDrive.drive(0, 0, rot, isFinished(), isFinished());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
