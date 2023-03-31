// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.board.DriveTrainTab;
import frc.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;

//Original, broken, will violently rotate out of control use Turn to angle profiled not this one, this is the original the profiled is based on

/** A command that will turn the robot to the specified angle. */
public class TurnToAngle extends PIDCommand {
  DriveSubsystem m_drive;
  double targetAngle;
  double m_initialHeading;
  private DriveTrainTab m_driveTab;
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive) {
    super(
        new PIDController(AutoConstants.kTurnP, AutoConstants.kTurnI, AutoConstants.kTurnD),
        // Close loop on heading
        drive::getAngle,
        // Set reference to target
        targetAngleDegrees + drive.getAngle(), //I think this may be better instead of resetting gyro in initialize
        // Pipe output to turn robot
        output -> drive.simpleArcadeDrive(0, (Math.abs(output) > 0.5) ? 0.5 * Math.signum(output): output, false), //Should set a max output of 0.5
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(AutoConstants.kTurnToleranceDeg, AutoConstants.kTurnRateToleranceDegPerS);
    // System.out.println("turn to anfle runniomng");
    m_driveTab = DriveTrainTab.getInstance();
    m_drive = drive;
  }

  @Override
  public void initialize() {
    super.initialize();
    //m_drive.resetGyro();
    getController().setPID(
      m_driveTab.getTurnkP(),
      m_driveTab.getTurnkI(),
      m_driveTab.getTurnkD());
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
