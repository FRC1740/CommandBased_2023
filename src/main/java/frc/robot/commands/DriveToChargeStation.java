// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToChargeStation extends CommandBase {
  /** Creates a new DriveToChargeStation.
   * Drives until Roll exceeds the Threshold (we are tilted)
  */
  
  private final DriveSubsystem m_drive;
  private double m_initialRoll;
  private double m_initialHeading;

  public DriveToChargeStation(DriveSubsystem drive) {
    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //  Consider setting m_initialRoll to 0 in case the command is started when already tilted
    m_initialRoll = m_drive.getRoll();
    m_initialHeading = m_drive.getAngle();
    if (Math.abs(m_initialRoll) > AutoConstants.kRollThresholdDegrees) {
      System.out.print("Starting assumption not met: Roll is " + m_initialRoll +
        " Threshold is " + AutoConstants.kRollThresholdDegrees);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleDelta = m_initialHeading - m_drive.getAngle();
    double rollDelta = m_drive.getRoll() - m_initialRoll;
    m_drive.arcadeDrive(Math.signum(rollDelta) * AutoConstants.kDriveToChargeStationPower,
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
      double rollDelta = m_drive.getRoll() - m_initialRoll;
      return (Math.abs(rollDelta) > AutoConstants.kRollThresholdDegrees);
    }    
}
