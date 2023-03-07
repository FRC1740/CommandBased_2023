// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.RobotShared;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.DriveOnAndBalanceChargeStation;
import frc.robot.commands.driver.AutoArmScoreHigh;
import frc.robot.commands.driver.ArmStow;
import frc.robot.commands.basic.ClawScore;
import frc.robot.commands.basic.IntakeDeploy;
import frc.robot.commands.basic.IntakeStow;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.math.util.Units;


public class RB_2_Pickup extends SequentialCommandGroup {
// import frc.robot.commands.basic.ClawScore;
  private DriveSubsystem m_drive;
  private RobotShared m_robotShared;

  public RB_2_Pickup() {

    m_robotShared = RobotShared.getInstance();
    m_drive = m_robotShared.getDriveSubsystem();

    addCommands (
      new AutoArmScoreHigh(),
      new WaitCommand(1),
      new ParallelDeadlineGroup (
        new ClawScore(),
        new WaitCommand(0.5)
        // Automatically calls scoreDone at end
      ),
      new ArmStow(),
      new DriveToDistance(Units.inchesToMeters(-95.575), m_drive),
      new IntakeDeploy(),
      new DriveToDistance(Units.inchesToMeters(-83.125), m_drive),
      new WaitCommand(0.5),
      new IntakeStow(),
      new DriveToDistance(Units.inchesToMeters(45.0), m_drive),
      new DriveOnAndBalanceChargeStation(true, m_drive)
    );

  }
}