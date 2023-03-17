// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotShared;
import frc.robot.commands.basic.ArmRotateRelative;
import frc.robot.commands.basic.ClawScore;

public class DunkScore extends SequentialCommandGroup {
  private RobotShared m_robotShared;
  
  /** Creates a new DunkScore. */
  public DunkScore() {

    m_robotShared = RobotShared.getInstance();

    addCommands(
      new ParallelDeadlineGroup(
        new WaitCommand(m_robotShared.calculateArmRotateRelativeDelay()),
        new ArmRotateRelative(m_robotShared.calculateRelativeArmSetpoint())),
        
      new ClawScore());
  }

}
