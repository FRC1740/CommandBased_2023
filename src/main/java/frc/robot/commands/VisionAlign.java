// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionAlign extends ProfiledPIDCommand {
  /** Creates a new VisionAlign. */
  LimeLightSubsystem m_LimeLight;
  public VisionAlign(DriveSubsystem drive, LimeLightSubsystem limeLight) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.0123,
            0.001,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)),
        // This should return the measurement
        limeLight::getXdeviation,
        // This should return the goal (can also be a constant)
        0,
        // This uses the output
        (output, setpoint) -> {drive.arcadeDrive(0, -output, false);
          // Use the output (and setpoint, if desired) here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.3);
    m_LimeLight = limeLight;
  }


  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_LimeLight.enableDriverCamera();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
