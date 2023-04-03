// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AprilTagAlign extends PIDCommand {
  PhotonVisionSubsystem m_PhotonVision;
  /** Creates a new AprilTagAlign. */
  public AprilTagAlign(DriveSubsystem drive, PhotonVisionSubsystem photonVision) {
    super(
        // The controller that the command will use
        new PIDController(
          // The PID gains
          0.0123, 
          0.001, 
          0),
        // This should return the measurement
        photonVision::getXdeviationAprilTag,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {drive.simpleArcadeDrive(0, -output, false);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
