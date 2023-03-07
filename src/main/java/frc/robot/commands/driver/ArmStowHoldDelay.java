// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.basic.ClawHold;
import frc.robot.commands.basic.ClawRollerStop;

public class ArmStowHoldDelay extends SequentialCommandGroup {

  /** Creates a new ArmStowHoldDelay. */
  public ArmStowHoldDelay() {

    addCommands(
      new ClawHold(),
      new WaitCommand(0.5),
      new ClawRollerStop(),
      new ArmStow());
  }

}
