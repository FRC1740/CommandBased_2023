// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {
  /** Creates a new DriveToDistance. */
  
  private final DriveSubsystem m_Drivesubsystem;
  private double goal = 0;
  private boolean Finished = false;
  private int targetInches = 0;
  public DriveToDistance(int inches, DriveSubsystem drive) {
    m_Drivesubsystem = drive;
    targetInches = inches;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Finished = false;
    goal = targetInches + m_Drivesubsystem.getAverageEncoderInches();
    System.out.println("goal " + goal);
    System.out.println("Encoder posotion " + m_Drivesubsystem.getAverageEncoder());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(goal < m_Drivesubsystem.getAverageEncoderInches()){
      //backwards
      while (!Finished){
        m_Drivesubsystem.arcadeDrive(-0.1, 0, false);
        if(m_Drivesubsystem.getAverageEncoderInches() <= goal){
          Finished = true;
        }
      }
    }else{
      //forwards
      while (!Finished){
        m_Drivesubsystem.arcadeDrive(0.1, 0, false);
        if(m_Drivesubsystem.getAverageEncoderInches() >= goal){
          Finished = true;
        }
      }
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
