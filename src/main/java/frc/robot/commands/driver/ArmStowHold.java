// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.basic.ClawHold;

public class ArmStowHold extends SequentialCommandGroup {

  /** Creates a new ArmStowAndHold. */
  public ArmStowHold() {

    addCommands(
      new ArmStow(),
      new ClawHold());
  }

}
