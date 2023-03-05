// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveOnAndBalanceChargeStation extends SequentialCommandGroup {
  /** Creates a new DriveOnAndBalanceChargeStation. */
  public DriveOnAndBalanceChargeStation(DriveSubsystem m_drive) {

    addCommands(
      new DriveToChargeStation(m_drive),
      new AutoBalancePID(m_drive)
    );
  }
}
