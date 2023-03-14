// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotShared;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;

public class ArmRotateRelative extends CommandBase {

  private ArmProfiledPIDSubsystem m_armProfiled;
  private RobotShared m_robotShared;
  private double m_relativeDegrees;

  /** Creates a new ArmRotateRelative. */
  public ArmRotateRelative(double relativeDegrees) {
    m_robotShared = RobotShared.getInstance();
    m_armProfiled = m_robotShared.getArmProfiledPIDSubsystem();
    m_relativeDegrees = relativeDegrees;
    addRequirements(m_armProfiled);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armProfiled.setGoal(m_armProfiled.getArmRotationDegrees() + m_relativeDegrees);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
