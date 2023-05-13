// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.constants.ArmConstants;
import frc.robot.RobotShared;
import frc.robot.commands.basic.ArmToSetpoint;
import frc.robot.commands.basic.ClawHold;
import frc.robot.commands.basic.TelescopeToSetpoint;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.TelescopePIDSubsystem;

public class AutoArmScoreHigh extends SequentialCommandGroup {
  RobotShared m_RobotShared;
  ArmProfiledPIDSubsystem m_arm;
  TelescopePIDSubsystem m_telescope;

  /** Creates a new AutoArmScoreHigh. */
  public AutoArmScoreHigh() {
    m_RobotShared = RobotShared.getInstance();
    m_arm = m_RobotShared.getArmProfiledPIDSubsystem();
    m_telescope = m_RobotShared.getTelescopePIDSubsystem();

    addCommands(
      new ArmToSetpoint(ArmConstants.AutoMode.HIGH),
      new WaitUntilCommand(m_arm::atGoal),
      new TelescopeToSetpoint(ArmConstants.AutoMode.HIGH)
      // new ClawHold()
    );
  }

}
