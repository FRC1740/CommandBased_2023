// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToChargeStation extends CommandBase {
  /** Creates a new DriveToDistance. */
  
  private final DriveSubsystem m_Drivesubsystem;
  private double goal = 0;
  private double startingRoll = 0;
  private boolean Finished = false;
  private int targetInches = 0;
  private double heading = 0;
  private double error = 0;

  public DriveToChargeStation(DriveSubsystem drive) {
    m_Drivesubsystem = drive;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Finished = false;
    startingRoll = m_Drivesubsystem.getRoll();
    heading = m_Drivesubsystem.getAngle();
    goal = targetInches + m_Drivesubsystem.getAverageEncoderInches();
    System.out.println("goal " + goal);
    System.out.println("Encoder posotion " + m_Drivesubsystem.getAverageEncoder());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Drivesubsystem.getRoll() >= startingRoll - 9 && m_Drivesubsystem.getRoll() <= startingRoll + 9 ){
       error = heading - m_Drivesubsystem.getAngle();
      m_Drivesubsystem.arcadeDrive(DriveConstants.kDriveToChargeStationPower, DriveConstants.kDriveCorrectionP * error, false);
    }else{
      Finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return Finished;
    }    
}
