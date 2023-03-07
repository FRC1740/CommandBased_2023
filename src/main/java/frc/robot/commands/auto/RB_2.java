// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.RobotShared;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.DriveOnAndBalanceChargeStation;
import frc.robot.commands.AutoBalancePID;
import frc.robot.commands.basic.*;
import frc.robot.commands.driver.*;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.math.util.Units;


public class RB_2 extends SequentialCommandGroup {
  private DriveSubsystem m_drive;
  private RobotShared m_robotShared;

  public RB_2() {
    
    m_robotShared = RobotShared.getInstance();
    m_drive = m_robotShared.getDriveSubsystem();

    addCommands (
      // new AutoArmScoreHigh(), // Move Arm & Telescope to high node position
      // new WaitCommand(1),
      // new ParallelDeadlineGroup (
      //   new WaitCommand(0.5),
      //   new ClawScore()
      //   // Automatically calls scoreDone at end
      // ),
      // new ArmStow(),
      // new DriveToDistance(Units.inchesToMeters(-32.0), m_drive),
      /// new DriveOnAndBalanceChargeStation(false, m_drive)
      new AutoBalancePID(m_drive)
   );

  }
}