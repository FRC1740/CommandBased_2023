// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.constants.ArmConstants;
import frc.robot.Paths;
import frc.robot.RobotShared;
import frc.robot.commands.basic.ClawScore;
import frc.robot.commands.driver.ArmStow;
import frc.robot.commands.driver.AutoArmScoreHigh;
import frc.robot.commands.driver.ToggleGamePiece;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.TelescopePIDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Blue_1_McDouble_Deluxe extends SequentialCommandGroup {
  private DriveSubsystem m_drive;
  private RobotShared m_robotShared;
  private ArmProfiledPIDSubsystem m_arm;
  private TelescopePIDSubsystem m_telescope;
  private PhotonVisionSubsystem m_photonVision;
  private Paths m_paths;
  
  /** Creates a new Blue_1_McDouble_Deluxe. */
  public Blue_1_McDouble_Deluxe() {

    m_robotShared = RobotShared.getInstance();
    m_drive = m_robotShared.getDriveSubsystem();
    m_arm = m_robotShared.getArmProfiledPIDSubsystem();
    m_telescope = m_robotShared.getTelescopePIDSubsystem();
    m_photonVision = m_robotShared.getPhotonVisionSubsystem();
    m_paths = m_robotShared.getPaths();

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand(getName() + " Started"),

      // Score the piece in the high position (Cube or Cone)
      // and stow the arm
      new AutoArmScoreHigh(), // Move Arm & Telescope to high node position
      new WaitCommand(m_robotShared.calculateAutoArmScoreDelay()),
      new ParallelDeadlineGroup (
        new WaitCommand(m_robotShared.calculateDunkScoreDelay()),
        new ClawScore()
        // Automatically calls scoreDone at end
      ),
      new ParallelDeadlineGroup (
        new WaitCommand(ArmConstants.kArmStowDelay),
        new ArmStow()
      ),
      new ToggleGamePiece(),
      m_drive.FollowPathWithEvents(m_paths.Blue_1_McDouble_Deluxe, true),
      new AutoArmScoreHigh(),
      new WaitCommand(m_robotShared.calculateAutoArmScoreDelay()),
      new ClawScore()
    );
  }
}
