// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class GettingInRangeAT extends CommandBase {

  double Kp = 0.3;

  double currentDistance;
  double desiredDistance;

  DriveSubsystem swerveDrive;
  Limelight limelight;

  /** Creates a new GettingInRangeAT. */
  public GettingInRangeAT(DriveSubsystem swerveDrive, Limelight limelight, double desiredDistance) {

    this.swerveDrive = swerveDrive;
    this.limelight = limelight;
    this.desiredDistance = desiredDistance;

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

    double[] botpose_targetspace = limelight.getBotPositionToTargetSpace();
    double currentDistance = -botpose_targetspace[2];

    double distanceError = desiredDistance - currentDistance;

    double speed = distanceError * Kp;

    if (limelight.getIsDetecting() == false) {
      speed = 0;
    }

    swerveDrive.drive(0, speed, 0, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.turnOffLED();
    swerveDrive.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
