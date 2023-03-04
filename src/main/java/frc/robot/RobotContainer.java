// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.board.ArmTab;
import frc.board.AutonomousTab;
import frc.board.ClawTab;
import frc.board.DriveTrainTab;
import frc.board.GroundIntakeTab;
import frc.board.RobotTab;
import frc.board.VisionTab;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SignalLEDSubsystem;
import frc.robot.subsystems.TelescopePIDSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private DriveSubsystem m_robotDrive;
  private ClawSubsystem m_claw;
  private ArmProfiledPIDSubsystem m_armProfiled;
  private TelescopePIDSubsystem m_telescope;
  private GroundIntakeSubsystem m_groundIntake;

  private RobotShared m_robotShared;
  private ArmTab m_armTab;
  private AutonomousTab m_autonomousTab;
  private ClawTab m_clawTab;
  private DriveTrainTab m_driveTrainTab;
  private GroundIntakeTab m_intakeTab;
  private RobotTab m_robotTab;
  private VisionTab m_visionTab;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_robotShared = RobotShared.getInstance();
    initSubsystems();
    initShuffleboard();
    configureButtonBindings();
  }

  private void initSubsystems() {
    m_robotDrive = m_robotShared.getDriveSubsystem();
    m_claw = m_robotShared.getClawSubsystem();
    m_armProfiled = m_robotShared.getArmProfiledPIDSubsystem();
    m_telescope = m_robotShared.getTelescopePIDSubsystem();
    m_groundIntake = m_robotShared.getGroundIntakeSubsystem();

    // Enable the Arm PID Subsystem
    m_armProfiled.enable();
    m_telescope.enable();
  }

  private void initShuffleboard() {
    m_armTab = ArmTab.getInstance();
    m_autonomousTab = AutonomousTab.getInstance();
    m_clawTab = ClawTab.getInstance();
    m_driveTrainTab = DriveTrainTab.getInstance();
    m_intakeTab = GroundIntakeTab.getInstance();
    m_robotTab = RobotTab.getInstance();
    m_visionTab = VisionTab.getInstance();
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    new ButtonBindings();
  }

  // Return the command to run in autonomous
  public Command getAutonomousCommand() {
      return m_autonomousTab.getAutonomousCommand();
  }

  public void autonomousInit() {
    if (DriverStation.isFMSAttached()) {
      m_robotDrive.burnFlash();
      m_claw.burnFlash();
      m_armProfiled.burnFlash();
      m_telescope.burnFlash();
      m_groundIntake.burnFlash();
    }
  }

}

