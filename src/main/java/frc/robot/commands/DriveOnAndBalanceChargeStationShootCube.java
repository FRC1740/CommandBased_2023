// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.constants.GroundIntakeConstants;
import frc.robot.commands.basic.IntakeEject;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveOnAndBalanceChargeStationShootCube extends SequentialCommandGroup {
  /** Creates a new DriveOnAndBalanceChargeStation. */
  public DriveOnAndBalanceChargeStationShootCube(boolean forward, DriveSubsystem m_drive) {

    addCommands(
      new DriveToChargeStation(forward, m_drive),
      new ParallelCommandGroup(new AutoBalancePID(m_drive), new IntakeEject(GroundIntakeConstants.kCubeEjectSpeed))
    );
  }
}
