// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.constants.ArmConstants;
import frc.constants.ArmConstants.AutoMode;
import frc.robot.RobotShared;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.TelescopePIDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceGamePiece extends SequentialCommandGroup {
  private RobotShared m_robotShared;

  /** Creates a new PlaceGamePiece. */

  public PlaceGamePiece(ArmProfiledPIDSubsystem m_arm, ClawSubsystem m_claw, TelescopePIDSubsystem m_telescope) {
    m_robotShared = RobotShared.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(AutoMode.HIGH))),
      new InstantCommand(() -> m_arm.setGoal(m_robotShared.calculateArmSetpoint(AutoMode.HIGH))),
      new WaitUntilCommand(m_arm::atGoal),
      new WaitUntilCommand(m_telescope::atSetpoint),
      new RunCommand(() -> m_claw.score()).withTimeout(1),
      new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kStowedPosition)),
      new InstantCommand(() -> m_arm.setGoal(ArmConstants.kStowedAngle)),
      new WaitUntilCommand(m_arm::atGoal),
      new WaitUntilCommand(m_telescope::atSetpoint));
    
  }
}

