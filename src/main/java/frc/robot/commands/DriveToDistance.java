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
  private double m_goal;
  private boolean m_Finished = false;
  private double m_targetMeters;
  private double m_heading;
  private double m_error = 0;

  public DriveToDistance(double meters, DriveSubsystem drive) {
    m_Drivesubsystem = drive;
    m_targetMeters = meters;
    m_goal = m_targetMeters + m_Drivesubsystem.getAverageEncoderMeters();
    m_Finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Finished = false;
    m_heading = m_Drivesubsystem.getAngle();
    m_goal = m_targetMeters + m_Drivesubsystem.getAverageEncoder();
    System.out.println("goal: " + m_goal);
    System.out.println("Encoder position " + m_Drivesubsystem.getAverageEncoder());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_error = m_heading - m_Drivesubsystem.getAngle();
    double distanceDelta = m_goal - m_Drivesubsystem.getAverageEncoder();
    System.out.println(distanceDelta);
    m_Drivesubsystem.simpleArcadeDrive(Math.signum(distanceDelta) * AutoConstants.kDriveToDistancePower, AutoConstants.kDriveCorrectionP * m_error*0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("done");
    System.out.println(interrupted);
    m_Drivesubsystem.simpleArcadeDrive(0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_goal - m_Drivesubsystem.getAverageEncoder()) < AutoConstants.epsilon);
    }    
}
