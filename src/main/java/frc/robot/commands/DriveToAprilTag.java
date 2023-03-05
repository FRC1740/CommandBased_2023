// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class DriveToAprilTag extends CommandBase {
  /** Creates a new DriveToAprilTag. */

  private DriveSubsystem m_drive;
  private PhotonVisionSubsystem m_vision;
  private double m_goal;

  public DriveToAprilTag(double targetDistanceFromTag, DriveSubsystem drive, PhotonVisionSubsystem vision) {
    m_drive = drive;
    m_vision = vision;
    m_goal = targetDistanceFromTag;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    addRequirements(m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deltaDistance = m_vision.getDistanceFromTag() - m_goal;
    m_drive.simpleArcadeDrive(Math.signum(deltaDistance) * AutoConstants.kDriveToAprilTag,
      AutoConstants.kAngleCorrectionP * m_vision.getXdeviationAprilTag(), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.simpleArcadeDrive(0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double deltaDistance = m_vision.getDistanceFromTag() - m_goal;
    return (Math.abs(deltaDistance) < AutoConstants.kDistanceEpsilonMeters);
  }
}
