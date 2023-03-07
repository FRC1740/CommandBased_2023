// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.constants.AutoConstants;

public class AutoBalancePID extends ProfiledPIDCommand {
  /** Creates a new AutoBalancePID.
   * Assumes starting Roll is greater than the tolerance for atGoal()
   */
  
  private DriveSubsystem m_drive;
  private double m_initialHeading;
  private static double m_headingDelta;

  public AutoBalancePID(DriveSubsystem drive) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            AutoConstants.kBalanceP,
            AutoConstants.kBalanceI,
            AutoConstants.kBalanceD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)),
        // This should return the measurement
        drive::getRoll,
        // This should return the goal (can also be a constant)
        AutoConstants.kLevel,
        // This uses the output
        (output, setpoint) -> drive.simpleArcadeDrive(-output, AutoConstants.kAngleCorrectionP * m_headingDelta, false),
        drive
        );

    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);

    // Configure additional PID options by calling `getController` here.
    getController()
      .setTolerance(AutoConstants.kBalanceToleranceDeg, AutoConstants.kTurnRateToleranceDegPerS);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public void initialize() {
    super.initialize();
    m_headingDelta = 0;
    m_initialHeading = m_drive.getAngle();
    if (m_drive.getRoll() < AutoConstants.kBalanceToleranceDeg) {
      System.out.print("Starting assumption not met: Roll is " + m_drive.getRoll() +
        " Tolerance is " + AutoConstants.kBalanceToleranceDeg);
    }
  }

  public void execute(){
    super.execute();
    m_headingDelta = m_initialHeading - m_drive.getAngle();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (getController().atGoal());
  }
}
