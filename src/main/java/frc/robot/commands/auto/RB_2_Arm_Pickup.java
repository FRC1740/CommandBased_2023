// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.constants.OIConstants.GamePiece;
import frc.robot.RobotShared;
import frc.robot.commands.*;
import frc.robot.commands.driver.*;
import frc.robot.commands.basic.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;


public class RB_2_Arm_Pickup extends SequentialCommandGroup {

  private DriveSubsystem m_drive;
  private PhotonVisionSubsystem m_photonVision;
  private RobotShared m_robotShared;

  public RB_2_Arm_Pickup(GamePiece piece) {

    m_robotShared = RobotShared.getInstance();
    m_drive = m_robotShared.getDriveSubsystem();
    m_photonVision = m_robotShared.getPhotonVisionSubsystem();

    Pose2d gamePiecePose = new Pose2d(7.07, 2.13, new Rotation2d()); // for blue side
    Pose2d returnBalancePose = new Pose2d(5.4, 2.13, new Rotation2d());

    addCommands (
      new PrintCommand(getName() + " Started"),

      // Score the piece in the high position (Cube or Cone)
      // and stow the arm
      new AutoArmScoreHigh(), // Move Arm & Telescope to high node position
      new WaitCommand(1.5),
      new ParallelDeadlineGroup (
        new WaitCommand(0.5),
        new ClawScore()
        // Automatically calls scoreDone at end
      ),
      new ParallelDeadlineGroup (
        new WaitCommand(0.5),
        new ArmStow()
      ),

      // Drive over the charge station and exit the community
      new DriveToDistanceVision(3.94, false, 0.3, m_drive, m_photonVision),

      new TurnTowardsPose(gamePiecePose, m_drive, m_photonVision),
      // Prepare the arm to pickup a piece and drive to the piece
      new AutoArmRetrieveLow(),
      new DriveTowardsPose(1.75,true,0.3, gamePiecePose, m_drive, m_photonVision),

      //stow the arm
      new ArmStow(),

      // return to the charge station and balance
      new DriveTowardsPose(1.75, false, 0.5, returnBalancePose, m_drive, m_photonVision),
      new DriveToDistance(-1, m_drive),
      new AutoBalancePID(m_drive),

      new PrintCommand(getName() + " Finished")
    );

  }
}