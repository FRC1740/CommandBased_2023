// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.constants.AutoConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalancePID extends ProfiledPIDCommand {
  /** Creates a new AutoBalancePID. */
  
  private DriveSubsystem m_drive;
  //private double initEncoderPos;
  private double heading;
  private static double error = 0;

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
        (output, setpoint) -> drive.arcadeDrive(-output, AutoConstants.kDriveCorrectionP * error, false),
        drive
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  getController()
      .setTolerance(AutoConstants.kBalanceToleranceDeg, AutoConstants.kTurnRateToleranceDegPerS);
      m_drive = drive;

  }
  @Override
  public void end(boolean interrupted) {

  }


  @Override
  public void initialize() {
    super.initialize();
    heading = m_drive.getAngle();
    //initEncoderPos = Math.abs(m_drive.getAverageEncoderInches());

  }
  public void execute(){
    super.execute();
    error = heading - m_drive.getAngle();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //28 is max inches traveled to avoid flying off the ramp
    return (getController().atGoal());/*||(initEncoderPos + 38) < Math.abs(m_drive.getAverageEncoderInches())*/
  }
}
