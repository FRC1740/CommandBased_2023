// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.ArmConstants;
import frc.robot.commands.basic.ArmToSetpoint;
import frc.robot.commands.basic.ClawHold;
import frc.robot.commands.basic.TelescopeToSetpoint;

public class AutoArmScoreHigh extends SequentialCommandGroup {

  /** Creates a new AutoArmScoreHigh. */
  public AutoArmScoreHigh() {

    addCommands(
      new ArmToSetpoint(ArmConstants.AutoMode.HIGH),
      new TelescopeToSetpoint(ArmConstants.AutoMode.HIGH)
      // new ClawHold()
    );
  }

}
