// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.board.DriveTrainTab;
import frc.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class TurnToAngleProfiled extends ProfiledPIDCommand {
  private DriveSubsystem m_drive;
  private DriveTrainTab m_driveTab;

  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnToAngleProfiled(double targetAngleDegrees, DriveSubsystem drive) {
    
    super(
        new ProfiledPIDController(
            AutoConstants.kTurnP,
            AutoConstants.kTurnI,
            AutoConstants.kTurnD,
            new TrapezoidProfile.Constraints(
                AutoConstants.kMaxTurnRateDegPerS,
                AutoConstants.kMaxTurnAccelerationDegPerSSquared)),
        // Close loop on heading
        drive::getAngle,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        (output, setpoint) -> drive.simpleArcadeDrive(0, output, false),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(AutoConstants.kTurnToleranceDeg, AutoConstants.kTurnRateToleranceDegPerS);
        m_drive = drive;

      m_driveTab = DriveTrainTab.getInstance();
  }

  @Override
  public void initialize() {
    super.initialize();
    m_drive.resetGyro();
    getController().setPID(
      m_driveTab.getTurnkP(),
      m_driveTab.getTurnkI(),
      m_driveTab.getTurnkD());
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
