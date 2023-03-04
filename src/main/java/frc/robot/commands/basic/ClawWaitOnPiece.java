// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotShared;
import frc.robot.subsystems.ClawSubsystem;

public class ClawWaitOnPiece extends CommandBase {

  private ClawSubsystem m_claw;
  private RobotShared m_robotShared;

  /** Creates a new ClawWaitOnPiece. */
  public ClawWaitOnPiece() {
    m_robotShared = RobotShared.getInstance();
    m_claw = m_robotShared.getClawSubsystem();
    addRequirements(m_claw);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_claw.pieceInClaw();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
