// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.board.DriveTrainTab;
import frc.constants.AutoConstants;

public class AutoBalancePIDLowPower extends ProfiledPIDCommand {
  /** Creates a new AutoBalancePID.
   * Assumes starting Roll is greater than the tolerance for atGoal()
   */
  private DriveSubsystem m_drive;
  private double m_initialHeading;
  private static double m_headingDelta;
  private DriveTrainTab m_driveTab;

  public AutoBalancePIDLowPower(DriveSubsystem drive) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.006,
            AutoConstants.kBalanceI,
            AutoConstants.kBalanceD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(999, 999)),
        // This should return the measurement
        drive::getPitch,
        // This should return the goal (can also be a constant)
        AutoConstants.kLevel,
        // This uses the output
        (output, setpoint) -> drive.simpleArcadeDrive(output, AutoConstants.kAngleCorrectionP * m_headingDelta, false),
        drive
        );

    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);

    // Configure additional PID options by calling `getController` here.
    getController()
      .setTolerance(AutoConstants.kBalanceToleranceDeg, AutoConstants.kTurnRateToleranceDegPerS);
    
    m_driveTab = DriveTrainTab.getInstance();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public void initialize() {
    super.initialize();
    m_headingDelta = 0;
    m_initialHeading = m_drive.getAngle();
    if (m_drive.getPitch() < AutoConstants.kBalanceToleranceDeg) {
      System.out.print("Starting assumption not met: Roll is " + m_drive.getPitch() +
        " Tolerance is " + AutoConstants.kBalanceToleranceDeg);
    }
  }

  public void execute() {
    // getController().setPID(
    //   m_driveTab.getAutoBalancekP(),
    //   m_driveTab.getAutoBalancekI(),
    //   m_driveTab.getAutoBalancekD());
    super.execute();
    m_headingDelta = m_initialHeading - m_drive.getAngle();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
