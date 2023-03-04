// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {
  /** Creates a new DriveToDistance. */
  
  private final DriveSubsystem m_Drivesubsystem;
  private double m_goal = 0;
  private boolean m_Finished = false;
  private double m_targetMeters = 0;
  private double m_heading = 0;
  private double m_error = 0;

  public DriveToDistance(double meters, DriveSubsystem drive) {
    m_Drivesubsystem = drive;
    m_targetMeters = meters;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Finished = false;
    m_heading = m_Drivesubsystem.getAngle();
    m_goal = m_targetMeters + m_Drivesubsystem.getAverageEncoderMeters();
    System.out.println("goal: " + m_goal);
    System.out.println("Encoder position " + m_Drivesubsystem.getAverageEncoder());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_error = m_heading - m_Drivesubsystem.getAngle();
    if(m_goal < m_Drivesubsystem.getAverageEncoderMeters()){
      m_Drivesubsystem.arcadeDrive(-AutoConstants.kDriveToDistancePower, AutoConstants.kDriveCorrectionP * m_error, false);
    }else{
      m_Drivesubsystem.arcadeDrive(AutoConstants.kDriveToDistancePower, AutoConstants.kDriveCorrectionP * m_error, false);
    }
    if(m_Drivesubsystem.getAverageEncoderMeters() >= m_goal - .1 && m_Drivesubsystem.getAverageEncoderMeters() <= m_goal + .1 ){
      m_Finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return m_Finished;
    }    
}
