// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.constants.ArmConstants.AutoMode;
import frc.robot.RobotShared;
import frc.robot.subsystems.TelescopePIDSubsystem;

public class TelescopeToSetpoint extends CommandBase {

  private TelescopePIDSubsystem m_telescope;
  private RobotShared m_robotShared;
  private AutoMode m_mode;

  /** Creates a new TelescopeToSetpoint. */
  public TelescopeToSetpoint(AutoMode mode) {
    m_mode = mode;
    m_robotShared = RobotShared.getInstance();
    m_telescope = m_robotShared.getTelescopePIDSubsystem();

    addRequirements(m_telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(m_mode));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
