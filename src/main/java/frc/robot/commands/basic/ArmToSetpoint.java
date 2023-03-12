// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.constants.ArmConstants.AutoMode;
import frc.robot.RobotShared;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;

public class ArmToSetpoint extends CommandBase {

  private ArmProfiledPIDSubsystem m_armProfiled;
  private RobotShared m_robotShared;
  private AutoMode m_mode;

  /** Creates a new ArmToSetpoint. */
  public ArmToSetpoint(AutoMode mode) {
    m_mode = mode;
    m_robotShared = RobotShared.getInstance();
    m_armProfiled = m_robotShared.getArmProfiledPIDSubsystem();
    addRequirements(m_armProfiled);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(m_mode));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
