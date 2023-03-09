// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SimpleBalance extends CommandBase {
  /** Creates a new SimpleBalance.
   * 
   * Consists of a simple state machine
   * CLIMBING- a few seconds at +9 or -9 determines direction
   * After direction is determined
   * COUNTING- each time rate in the opposite direction exceeds 30 degrees/second, increment counter
   * When threshold is reached, DONE
   */
  
  private final DriveSubsystem m_drive;
  private double m_direction;
  private double m_initialHeading;
  private int m_count;

  private enum State {
    CLIMBING,
    CREEPING
  }
  private State m_state;

  public SimpleBalance(boolean forward, DriveSubsystem drive) {
    m_drive = drive;
    m_direction = forward ? 1.0 : -1.0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = State.CLIMBING;
    m_initialHeading = m_drive.getAngle();
    m_count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleDelta = m_initialHeading - m_drive.getAngle();
    if (m_state == State.CLIMBING) {
      m_drive.simpleArcadeDrive(m_direction * AutoConstants.kSimpleBalanceClimbingPower,
        AutoConstants.kAngleCorrectionP * angleDelta, false);
      if (Math.abs(m_drive.getPitch()) > AutoConstants.kSimpleBalanceClimbThreshold) {
        m_count++;
        if (m_count > AutoConstants.kSimpleBalanceClimbingCount) {
          m_count = 0;
          m_state = State.CREEPING;
        }
      }
    }
    else { // CREEPING
      double pitchRate = m_drive.getRawGyroX(); // degrees/sec
      m_drive.simpleArcadeDrive(m_direction * AutoConstants.kSimpleBalanceCreepingPower,
        AutoConstants.kAngleCorrectionP * angleDelta, false);
      if (Math.abs(pitchRate) > AutoConstants.kSimpleBalanceTippingRateThreshold) {
        m_count++;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.simpleArcadeDrive(0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((m_state == State.CLIMBING) && (m_count > AutoConstants.kSimpleBalanceTipppingCount));
  }    
}
