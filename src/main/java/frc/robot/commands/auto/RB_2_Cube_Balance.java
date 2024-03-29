// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.constants.ArmConstants;
import frc.constants.GroundIntakeConstants;
import frc.constants.OIConstants.GamePiece;
import frc.robot.RobotShared;
import frc.robot.commands.*;
import frc.robot.commands.driver.*;
import frc.robot.commands.basic.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;


public class RB_2_Cube_Balance extends SequentialCommandGroup {

  private DriveSubsystem m_drive;
  private PhotonVisionSubsystem m_photonVision;
  private RobotShared m_robotShared;

  public RB_2_Cube_Balance() {

    m_robotShared = RobotShared.getInstance();
    m_drive = m_robotShared.getDriveSubsystem();
    m_photonVision = m_robotShared.getPhotonVisionSubsystem();

    Pose2d gamePiecePose = new Pose2d(7.07, 2.13, new Rotation2d()); // for blue side
    Pose2d returnBalancePose = new Pose2d(5.4, 2.13, new Rotation2d());

    addCommands (
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

      new IntakeDeploy(GroundIntakeConstants.kCubeIntakeSpeed),
      new DriveToDistance(Units.inchesToMeters(-198), 0.4, m_drive),
      new WaitCommand(.25), // FIXME: guestimate time
      new IntakeStow(),
      new TurnToAngle(180, m_drive),
      new DriveToDistance(Units.inchesToMeters(-118), 0.4, m_drive),
      
      new ParallelDeadlineGroup (
        new DriveToDistance(Units.inchesToMeters(-12), 0.15, m_drive),
        // Yeet the cube
        new IntakeEject(GroundIntakeConstants.kCubeEjectSpeed)
      ),
      new IntakeStop(),
      // Drive over the charge station and exit the community
      new WaitCommand(0.1),
      // Balance on the charge station
      new AutoBalancePIDLowPower(m_drive)

      // new DriveOnAndBalanceChargeStationShootCube(false, m_drive),
      // new IntakeStop(),
      // new PrintCommand(getName() + " Finished")
    );

  }
}