// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotShared;
import frc.robot.subsystems.ClawSubsystem;

public class ClawRetrieve extends CommandBase {

  private ClawSubsystem m_claw;
  private RobotShared m_robotShared;

  /** Creates a new ClawRetrieve. */
  public ClawRetrieve() {
    m_robotShared = RobotShared.getInstance();
    m_claw = m_robotShared.getClawSubsystem();
    addRequirements(m_claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_claw.retrieve();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
