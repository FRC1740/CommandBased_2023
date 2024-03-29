// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.Paths;
import frc.robot.RobotShared;
import frc.constants.ArmConstants;
import frc.constants.GroundIntakeConstants;
import frc.constants.ArmConstants.AutoMode;

import frc.robot.commands.*;
import frc.robot.commands.basic.*;
import frc.robot.commands.driver.*;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.TelescopePIDSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.math.util.Units;


public class Blue_3_McDouble_Deluxe extends SequentialCommandGroup {
  private DriveSubsystem m_drive;
  private RobotShared m_robotShared;
  private ArmProfiledPIDSubsystem m_arm;
  private TelescopePIDSubsystem m_telescope;
  private PhotonVisionSubsystem m_photonVision;
  private Paths m_paths;

  public Blue_3_McDouble_Deluxe() {

    m_robotShared = RobotShared.getInstance();
    m_drive = m_robotShared.getDriveSubsystem();
    m_arm = m_robotShared.getArmProfiledPIDSubsystem();
    m_telescope = m_robotShared.getTelescopePIDSubsystem();
    m_photonVision = m_robotShared.getPhotonVisionSubsystem();
    m_paths = m_robotShared.getPaths();

    addCommands (
      new PrintCommand(getName() + " Started"),

      // Score the piece in the high position (Cube or Cone)
      // and stow the arm
      new AutoArmScoreHigh(), // Move Arm & Telescope to high node position
      new WaitCommand(m_robotShared.calculateAutoArmScoreDelay()),
      new ParallelDeadlineGroup (
        new WaitCommand(m_robotShared.calculateDunkScoreDelay()),
        new ClawScore()
        // Automatically calls scoreDone at end
      ),
      new ParallelDeadlineGroup (
        new WaitCommand(ArmConstants.kArmStowDelay),
        new ArmStow()
      ),

     m_drive.FollowPath(m_paths.Blue_3_McDouble_Deluxe_pt1, true),
     new TurnToAngle(180, m_drive),
     new ToggleGamePiece(),
     new AutoArmRetrieveLow(),
     m_drive.FollowPath(m_paths.Blue_3_McDouble_Deluxe_pt2, false),
     new ArmStow(),
     new ClawRollerStop(),
     new TurnToAngle(180, m_drive),
     m_drive.FollowPathVision(m_paths.Blue_3_McDouble_Deluxe_pt3, false),
     
     new AprilTagAlign(m_drive, m_photonVision),

      new AutoArmScoreHigh(), // Move Arm & Telescope to high node position
      new WaitCommand(m_robotShared.calculateAutoArmScoreDelay()),
      new ParallelDeadlineGroup (
        new WaitCommand(m_robotShared.calculateDunkScoreDelay()),
        new ClawScore()
        // Automatically calls scoreDone at end
      )
    );
    
  }
}