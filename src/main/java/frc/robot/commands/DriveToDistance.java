// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {
  /** Creates a new DriveToDistance. */
  
  private final DriveSubsystem m_drive;
  private double m_meters;
  private double m_goal;
  private double m_initialHeading;
  private double m_power;

  public DriveToDistance(double meters, double power,DriveSubsystem drive) {
    m_meters = meters;
    m_drive = drive;
    m_power = power;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  public DriveToDistance(double meters, DriveSubsystem drive) {
    m_meters = meters;
    m_drive = drive;
    m_power = 0.5;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialHeading = m_drive.getAngle();
    // Goal = distance to travel + current position
    m_goal = m_meters + m_drive.getAverageEncoderMeters();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleDelta = m_initialHeading - m_drive.getAngle();
    double distanceDelta = m_goal - m_drive.getAverageEncoderMeters();
    m_drive.simpleArcadeDrive(Math.signum(distanceDelta) * m_power,
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
    double distanceDelta = m_goal - m_drive.getAverageEncoderMeters();
    return (Math.abs(distanceDelta) < AutoConstants.kAutoDriveToleranceMeters);
  }    
}
