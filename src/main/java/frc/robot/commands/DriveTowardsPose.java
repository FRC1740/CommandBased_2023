// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class DriveTowardsPose extends CommandBase {
  /** Creates a new DriveTowardsPose. */
  
  private final DriveSubsystem m_drive;
  private final PhotonVisionSubsystem m_vision;
  private double m_meters;
  private double m_power;
  private Pose2d m_initialPose;
  private double m_direction;
  private Pose2d m_targetPose;

  public DriveTowardsPose(double meters, double power, Pose2d targetPose, DriveSubsystem drive, PhotonVisionSubsystem vision) {
    m_meters = meters;
    m_direction = Math.signum(meters);
    m_targetPose = targetPose;
    m_drive = drive;
    m_vision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialPose = m_drive.getEstimatedVisionPose();
    // Goal = distance to travel + current position
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleDelta = m_vision.getYawToPose(m_targetPose).getDegrees();
    //m_drive.getEstimatedVisionPose().getRotation().getDegrees() - m_initialPose.getRotation().plus(m_vision.getYawToPose(m_targetPose)).getDegrees();
    m_drive.simpleArcadeDrive(m_direction * m_power,
      AutoConstants.kAngleCorrectionP * angleDelta, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.simpleArcadeDrive(0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Eror = goal - current position
    double distanceDelta = m_meters - m_vision.getDistanceToPose(m_initialPose);
    return (Math.abs(distanceDelta) < AutoConstants.kAutoDriveToleranceMeters);
  }    
}
