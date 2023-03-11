// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToChargeStation extends CommandBase {
  /** Creates a new DriveToChargeStation.
   * Drives until Pitch exceeds the Threshold (we are tilted)
  */
  
  private final DriveSubsystem m_drive;
  private double m_initialPitch;
  private double m_initialHeading;
  private double m_direction;

  public DriveToChargeStation(Boolean forward, DriveSubsystem drive) {
    m_drive = drive;
    if (forward){
      m_direction = 1;
    } else {
      m_direction = -1;
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //  Consider setting m_initialPitch to 0 in case the command is started when already tilted
    // m_initialPitch = m_drive.getPitch();
    m_initialPitch = 0;
    m_initialHeading = m_drive.getAngle();
    if (Math.abs(m_initialPitch) > AutoConstants.kPitchThresholdDegrees) {
      System.out.print("Starting assumption not met: Pitch is " + m_initialPitch +
        " Threshold is " + AutoConstants.kPitchThresholdDegrees);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleDelta = m_initialHeading - m_drive.getAngle();
    double pitchDelta = m_drive.getPitch() - m_initialPitch;
    m_drive.arcadeDrive(Math.signum(pitchDelta) * AutoConstants.kDriveToChargeStationPower * m_direction,
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
      double pitchDelta = m_drive.getPitch() - m_initialPitch;
      return (Math.abs(pitchDelta) > AutoConstants.kPitchThresholdDegrees);
    }    
}
