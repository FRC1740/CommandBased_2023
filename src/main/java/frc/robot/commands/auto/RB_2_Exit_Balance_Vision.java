// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.RobotShared;
import frc.robot.commands.*;
import frc.robot.commands.basic.*;
import frc.robot.commands.driver.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RB_2_Exit_Balance_Vision extends SequentialCommandGroup {
  private DriveSubsystem m_drive;
  private PhotonVisionSubsystem m_photonVision;
  private RobotShared m_robotShared;
  

  public RB_2_Exit_Balance_Vision() {

    m_robotShared = RobotShared.getInstance();
    m_drive = m_robotShared.getDriveSubsystem();
    m_photonVision = m_robotShared.getPhotonVisionSubsystem();

    addCommands (
      new PrintCommand(getName() + " Started"),

      // Score the piece in the high position (Cube or Cone)
      // and stow the arm

      // new AutoArmScoreHigh(), // Move Arm & Telescope to high node position
      // new WaitCommand(1.5),
      // new ParallelDeadlineGroup (
      //   new WaitCommand(0.5),
      //   new ClawScore()
      //   // Automatically calls scoreDone at end
      // ),
      // new ParallelDeadlineGroup (
      //   new WaitCommand(0.5),
      //   new ArmStow()
      // ),

      // Drive over the charge station and exit the community
      new DriveToDistanceVision(3.94, false, 0.3, m_drive, m_photonVision),
      //new TurnToAngle(180, m_drive),
      new DriveToDistance(-1.3, m_drive),
      // Balance on the charge station
      new AutoBalancePID(m_drive),
      
      

      new PrintCommand(getName() + " Finished")
    );

  }
}
