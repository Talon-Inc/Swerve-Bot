// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignAtAprilTag extends CommandBase {

  PIDController speedXController, speedYController;
  PIDController speedRotController;

  DriveSubsystem swerveDrive;
  DriveSubsystem odometry;
  Limelight limelight;

  double targetX, targetY;

  ProfiledPIDController speedController;

  /** Creates a new AlignAtAprilTag. */
  public AlignAtAprilTag(DriveSubsystem swerveDrive, Limelight limelight, double targetX, double targetY) {

    this.swerveDrive = swerveDrive;
    this.limelight = limelight;
    this.targetX = targetX;
    this.targetY = targetY;

    speedController = new ProfiledPIDController(1, 0, 0, new Constraints(0, 0));
    speedRotController = new PIDController(1, 0, 0);
    speedRotController.enableContinuousInput(-180, 180);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive, limelight);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d robotPose = swerveDrive.getPose();

    Pose2d targetPose;
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
