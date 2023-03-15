// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.constants.ArmConstants;
import frc.robot.RobotShared;
import frc.robot.commands.*;
import frc.robot.commands.basic.*;
import frc.robot.commands.driver.*;
import frc.robot.subsystems.DriveSubsystem;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RB_2_Exit_Balance extends SequentialCommandGroup {
  private DriveSubsystem m_drive;
  private RobotShared m_robotShared;

  public RB_2_Exit_Balance() {

    m_robotShared = RobotShared.getInstance();
    m_drive = m_robotShared.getDriveSubsystem();

    addCommands (
      new PrintCommand(getName() + " Started"),

      // Score the piece in the high position (Cube or Cone)
      // and stow the arm
      new AutoArmScoreHigh(), // Move Arm & Telescope to high node position
      new WaitCommand(m_robotShared.calculateAutoArmScoreDelay()),
      new ParallelDeadlineGroup (
        new WaitCommand(m_robotShared.calculateDunkScoreDelay()),
        new DunkScore()
        // Automatically calls scoreDone at end
      ),
      new ParallelDeadlineGroup (
        new WaitCommand(ArmConstants.kArmStowDelay),
        new ArmStow()
      ),

      // Drive over the charge station and exit the community
      new DriveToDistance(Units.inchesToMeters(136.0), m_drive),

      // Balance on the charge station
      new DriveOnAndBalanceChargeStation(true, m_drive),

      new PrintCommand(getName() + " Finished")
    );

  }
}
