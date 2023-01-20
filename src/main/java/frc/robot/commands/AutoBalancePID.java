// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalancePID extends ProfiledPIDCommand {
  /** Creates a new AutoBalancePID. */
  private XboxController m_codriverController;
  public AutoBalancePID(DriveSubsystem drive, XboxController coDriveController) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            DriveConstants.kBalanceP,
            DriveConstants.kBalanceI,
            DriveConstants.kBalanceD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)),
        // This should return the measurement
        drive::getRoll,
        // This should return the goal (can also be a constant)
        DriveConstants.kLevel,
        // This uses the output
        (output, setpoint) -> drive.arcadeDrive(-output, 0, false),
        drive
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  getController()
      .setTolerance(DriveConstants.kBalanceToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  m_codriverController = coDriveController;
  }
  @Override
  public void end(boolean interrupted) {
    m_codriverController.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_codriverController.setRumble(RumbleType.kBothRumble, 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
