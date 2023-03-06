// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotShared;
import frc.robot.commands.basic.ClawScore;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.math.util.Units;


public class Auto_RB_2 extends SequentialCommandGroup {
  private DriveSubsystem m_drive;
  private RobotShared m_robotShared;

  public Auto_RB_2() {
    
    m_robotShared = RobotShared.getInstance();
    m_drive = m_robotShared.getDriveSubsystem();

    addCommands (
      new ClawScore(),
      new DriveToDistance(Units.inchesToMeters(32.0), m_drive),
      new DriveOnAndBalanceChargeStation(true, m_drive)
    );

  }
}