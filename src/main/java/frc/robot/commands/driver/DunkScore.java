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

  /** Creates a new DunkScore. */
  public DunkScore() {

    addCommands(
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new ArmRotateRelative(RobotShared.getInstance().calculateRelativeArmSetpoint())),
      new ClawScore());
  }

}
