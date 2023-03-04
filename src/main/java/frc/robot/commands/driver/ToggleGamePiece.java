// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotShared;

public class ToggleGamePiece extends CommandBase {

  private RobotShared m_robotShared;

  /** Creates a new ToggleGamePiece. */
  public ToggleGamePiece() {
    m_robotShared = RobotShared.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotShared.toggleGamePiece();;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
