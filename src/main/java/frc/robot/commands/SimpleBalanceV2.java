// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.constants.AutoConstants;
import frc.constants.DriveConstants;
import frc.robot.RobotShared;
import frc.robot.subsystems.DriveSubsystem;


public class SimpleBalanceV2 extends CommandBase {
  RobotShared m_RobotShared = RobotShared.getInstance();
  DriveSubsystem m_drive;
  Double m_initialHeading;
  Double m_headingError;
  /** Creates a new SimpleBalanceV2. */
  public SimpleBalanceV2() {
    m_drive = m_RobotShared.getDriveSubsystem();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialHeading = m_drive.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_headingError = m_initialHeading - m_drive.getAngle();
    m_drive.simpleArcadeDrive(AutoConstants.kSimpleBalanceClimbingPower * -Math.signum(m_drive.getPitch()), m_headingError * AutoConstants.kAngleCorrectionP, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.simpleArcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getRawGyroX()) > AutoConstants.kSimpleBalanceTippingRateThreshold;
  }
}
