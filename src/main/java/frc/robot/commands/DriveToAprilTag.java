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
  private double goalDistanceFromTag;
  private boolean atGoal = false;

  public DriveToAprilTag(double targetDistanceFromTag, DriveSubsystem drive, PhotonVisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drive = drive;
    m_vision = vision;
    goalDistanceFromTag = targetDistanceFromTag;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_vision.getDistanceFromTag() > goalDistanceFromTag){
      m_drive.simpleArcadeDrive(0.3, AutoConstants.kDriveCorrectionP * m_vision.getXdeviationAprilTag(), false);
    } else {
      atGoal = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atGoal;
  }
}
