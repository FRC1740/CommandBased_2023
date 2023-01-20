// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveSequential extends SequentialCommandGroup {
  /** Creates a new AutoDrive. */
  public AutoDriveSequential(DriveSubsystem drive) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToDistance(24, drive), //Drive forward 24 inches
      new TurnToAngleProfiled(90, drive),

      new DriveToDistance(24, drive ), //Drive forward 24 inches
      new TurnToAngleProfiled(90, drive),

      new DriveToDistance(24, drive ), //Drive forward 24 inches
      new TurnToAngleProfiled(90, drive),
      
      new DriveToDistance(24, drive ), //Drive forward 24 inches
      new TurnToAngleProfiled(90, drive)
    );
  }
}