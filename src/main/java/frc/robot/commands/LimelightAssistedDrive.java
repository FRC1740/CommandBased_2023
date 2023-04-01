// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.constants.DriveConstants;
import frc.robot.RobotShared;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class LimelightAssistedDrive extends CommandBase {
  private LimeLightSubsystem m_limelight;
  private DriveSubsystem m_drive;
  private RobotShared m_RobotShared;
  private CommandXboxController m_driverController;
  private SlewRateLimiter m_speedLimiter;
  /** Creates a new LimelightAssistedDrive. */
  public LimelightAssistedDrive() {
    m_RobotShared = RobotShared.getInstance();
    m_drive = m_RobotShared.getDriveSubsystem();
    m_limelight = m_RobotShared.getLimeLightSubsystem();
    m_driverController = m_RobotShared.getDriverController();
    m_speedLimiter = new SlewRateLimiter(DriveConstants.kDrivePositiveRateLimit, DriveConstants.kDriveNegativeRateLimit, 0);
    addRequirements(m_drive, m_limelight);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.enableVisionProcessing();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_drive.simpleArcadeDrive(m_speedLimiter.calculate(m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis()),
     ((m_limelight.getXdeviation() == 0) ? m_driverController.getLeftX() : m_limelight.getXdeviation()+DriveConstants.kLimelightOffsetDegrees) * 0.02, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.enableDriverCamera();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
