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
  private Double m_setpoint;

  /** Creates a new TelescopeToSetpoint. */
  public TelescopeToSetpoint(AutoMode mode) {
    m_robotShared = RobotShared.getInstance();
    m_setpoint = m_robotShared.calculateTeleSetpoint(mode);
    m_telescope = m_robotShared.getTelescopePIDSubsystem();

    addRequirements(m_telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_telescope.setSetpoint(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
