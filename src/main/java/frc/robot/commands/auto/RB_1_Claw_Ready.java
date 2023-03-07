// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.RobotShared;
import frc.constants.ArmConstants.AutoMode;

import frc.robot.commands.DriveToDistance;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.commands.basic.*;
import frc.robot.commands.driver.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.TelescopePIDSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.math.util.Units;


public class RB_1_Claw_Ready extends SequentialCommandGroup {
  private DriveSubsystem m_drive;
  private RobotShared m_robotShared;
  private ArmProfiledPIDSubsystem m_arm;
  private TelescopePIDSubsystem m_telescope;

  public RB_1_Claw_Ready() {

    m_robotShared = RobotShared.getInstance();
    m_drive = m_robotShared.getDriveSubsystem();
    m_arm = m_robotShared.getArmProfiledPIDSubsystem();
    m_telescope = m_robotShared.getTelescopePIDSubsystem();

    addCommands (
      new AutoArmScoreHigh(), // Move Arm & Telescope to high node position
      new ClawScore(),
      new ArmStow(),
      new DriveToDistance(Units.inchesToMeters(-155.875), m_drive),
      new TurnToAngleProfiled(-166.883, m_drive),
      new DriveToDistance(Units.inchesToMeters(20.0), m_drive),
      new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(AutoMode.FLOOR))),
      new InstantCommand(() -> m_arm.setGoal(m_robotShared.calculateArmSetpoint(AutoMode.FLOOR))),
      new WaitUntilCommand(m_arm::atGoal),
      new WaitUntilCommand(m_telescope::atSetpoint)
    );

  }
}