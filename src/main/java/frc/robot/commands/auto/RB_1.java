// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.RobotShared;
import frc.robot.commands.*;
import frc.robot.commands.basic.*;
import frc.robot.commands.driver.*;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.util.Units;


public class RB_1 extends SequentialCommandGroup {
  private DriveSubsystem m_drive;
  private RobotShared m_robotShared;

  public RB_1() {

    m_robotShared = RobotShared.getInstance();
    m_drive = m_robotShared.getDriveSubsystem();

    addCommands (
      new PrintCommand(getName() + " Started"),

      // Score the piece in the high position (Cube or Cone)
      // and stow the arm
      new AutoArmScoreHigh(), // Move Arm & Telescope to high node position
      new WaitCommand(1.5),
      new ParallelDeadlineGroup (
        new WaitCommand(0.5),
        new ClawScore()
        // Automatically calls scoreDone at end
      ),
      new ParallelDeadlineGroup (
        new WaitCommand(0.5),
        new ArmStow()
      ),

      // Drive out of the community and park in front of piece
      new DriveToDistance(Units.inchesToMeters(-155.875), m_drive),
      new TurnToAngle(-166.783, m_drive), // 180 - 13.217 (We want the claw ready)
      new DriveToDistance(Units.inchesToMeters(20), m_drive), // Was -57.35

      new PrintCommand(getName() + " Finished")
    );
      
  }
}
