// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.AutoConstants;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TelescopePIDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SubStationSideAuto extends SequentialCommandGroup {
  /** Creates a new SubStationSideAuto. */
  public SubStationSideAuto(ArmProfiledPIDSubsystem m_arm, ClawSubsystem m_claw, DriveSubsystem m_drive, TelescopePIDSubsystem m_telescope) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PlaceGamePiece(m_arm, m_claw, m_telescope),
      new DriveToDistance(AutoConstants.kSubStationSideDriveDistance, m_drive)
    );
  }
}
