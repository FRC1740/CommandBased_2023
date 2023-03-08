// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotShared;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;

public class ArmManual extends CommandBase {

  private ArmProfiledPIDSubsystem m_armProfiled;
  private CommandXboxController m_controller;
  private RobotShared m_robotShared;

  /** Creates a new ArmManual. */
  public ArmManual(CommandXboxController controller) {
    m_controller = controller;
    m_robotShared = RobotShared.getInstance();
    m_armProfiled = m_robotShared.getArmProfiledPIDSubsystem();
    addRequirements(m_armProfiled);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armProfiled.manualArmRotate(m_controller.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armProfiled.manualDone();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
