// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotShared;
import frc.robot.subsystems.GroundIntakeSubsystem;

public class IntakeGrasp extends CommandBase {

  private GroundIntakeSubsystem m_intake;
  private RobotShared m_robotShared;
  private double m_graspSpeed;

  /** Creates a new IntakeGrasp. */
  public IntakeGrasp(double graspSpeed) {
    m_robotShared = RobotShared.getInstance();
    m_intake = m_robotShared.getGroundIntakeSubsystem();
    m_graspSpeed = graspSpeed;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.grasp(m_graspSpeed);
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
