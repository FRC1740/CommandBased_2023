// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import frc.constants.ArmConstants;
import frc.constants.DriveConstants;
import frc.constants.OIConstants;

import frc.robot.commands.AutoBalancePID;
import frc.robot.commands.DriveToDistance;
//import frc.robot.commands.SequentialVisionAlign;
//import frc.robot.commands.DriveOnAndBalanceChargeStation;
//import frc.robot.commands.RotateArmToAngle;
//import frc.robot.commands.AprilTagAlign;
import frc.robot.commands.TurnToAngleProfiled;

import frc.robot.subsystems.LimeLight;
//import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ProfiledPIDArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.ArmExtensionPID;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.SignalLEDs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LimeLight m_limelight = new LimeLight();
  //private final PhotonVision m_photonVision = new PhotonVision();
  protected final Claw m_claw = new Claw();
  protected final ArmPIDSubsystem m_arm = new ArmPIDSubsystem();
  protected final ProfiledPIDArmSubsystem m_armProfiled = new ProfiledPIDArmSubsystem();
  protected final ArmExtensionPID m_telescope = new ArmExtensionPID();
  protected final GroundIntake m_groundIntake = new GroundIntake();
  private final SignalLEDs m_signalLEDs = new SignalLEDs();

  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_codriverController = new CommandXboxController(OIConstants.kCoDriverControllerPort);
  
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  // auto command
  // Note: The autocommand is dynamically retrieved in getAutonomousCommand.
  // This member variable is currently unused
  // private Command m_autoCommand = m_AutoChooser.getSelected();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drivem_robotDrive
    m_robotDrive.setDefaultCommand(
      // "Mario-Cart" drive: Triggers are gas and brake. Right stick turns left/right
      // Triggers are Axis 2; RightStick X is axis 3
      // Note the constants defined in the wpi XboxController class DO NOT MATCH the DS axes
      new RunCommand(() ->
        m_robotDrive.arcadeDrive(m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis(),
        m_driverController.getLeftX(), true), m_robotDrive));
    
    m_AutoChooser.addOption("curvy path", m_robotDrive.FollowPath(PathPlanner.loadPath("Curvy Path",
        new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond,
          DriveConstants.kMaxAccelerationMetersPerSecondSquared)),
          true));

    m_AutoChooser.addOption("straight", m_robotDrive.FollowPath(PathPlanner.loadPath("Straight path",
        new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond,
          DriveConstants.kMaxAccelerationMetersPerSecondSquared)),
          true));

    m_AutoChooser.addOption("Short Straight path", m_robotDrive.FollowPath(PathPlanner.loadPath("Short_Straight_Path",
        new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond,
          DriveConstants.kMaxAccelerationMetersPerSecondSquared)),
          true));

    m_AutoChooser.addOption("more curvy path", m_robotDrive.FollowPath(PathPlanner.loadPath("more curvy path",
        new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond,
          DriveConstants.kMaxAccelerationMetersPerSecondSquared)),
          true));

    Shuffleboard.getTab("Autonomous").add(m_AutoChooser);
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    // bind_CircleTest();
    // bind_AutoDriveDistanceTest();
    // bind_ManualArmTest();
    // bind_ResetGyro();
    bind_GroundIntakeTest();
    // bind_Limelight();
    // bind_HalfSpeed();
    // bind_PathWeaver();
    // bind_CoAutoBalance();
    // bind_CoLightToggle();

    // // Enable the Arm PID Subsystem
    // // m_arm.enable();
    // m_armProfiled.enable();
    // m_telescope.enable();

    // bind_ArmPID();
    // bind_ArmAndTelescope();
    // bind_LedModeTest();
    // bind_LedSubsystemTest();
  }

  private void bind_CircleTest() {
    // Turn to -90 degrees with a profile when the Circle button is pressed, with a
    // 5 second timeout
    m_driverController.a()
      .onTrue(new TurnToAngleProfiled(90, m_robotDrive).withTimeout(5));
  }

  private void bind_AutoDriveDistanceTest() {
    // Auto-drive distance
    m_driverController.b()
      .onTrue(new DriveToDistance(5, m_robotDrive));
  }

  private void bind_ManualArmTest() {
    m_driverController.a()
      .onTrue(new InstantCommand(() -> m_armProfiled.manualArmRotateUp()))
      .onFalse(new InstantCommand(() -> m_armProfiled.manualDone()));

    m_driverController.b()
      .onTrue(new InstantCommand(() -> m_armProfiled.manualArmRotateDown()))
      .onFalse(new InstantCommand(() -> m_armProfiled.manualDone()));

    m_driverController.x()
      .onTrue(new InstantCommand(() -> m_telescope.manualTelescopeOut()))
      .onFalse(new InstantCommand(() -> m_telescope.manualDone()));

    m_driverController.y()
      .onTrue(new InstantCommand(() -> m_telescope.manualTelescopeIn()))
      .onFalse(new InstantCommand(() -> m_telescope.manualDone()));
  }

  private void bind_ResetGyro() {
    // Manually rsest the gyro
    m_driverController.start()
      .onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));
  }

  private void bind_GroundIntakeTest() {
    m_driverController.rightBumper()
      // This command must also stow the arm first!!
      .onTrue(new InstantCommand(() -> m_groundIntake.deploy()))
      .onFalse(new InstantCommand(() -> m_groundIntake.stow()));

    m_driverController.leftBumper()
      .onTrue(new InstantCommand(() -> m_groundIntake.eject()))
      .onFalse(new InstantCommand(() -> m_groundIntake.stow()));

    m_driverController.x()
      .onTrue(new InstantCommand(() -> m_groundIntake.setIntakeSpeed(0.1)));

    m_driverController.y()
      .onTrue(new InstantCommand(() -> m_groundIntake.setIntakeSpeed(-0.1)));
  }

  private void bind_Limelight() {
    m_driverController.y()
      .onTrue(new InstantCommand(() -> m_limelight.enableVisionProcessing()));
  }

  private void bind_HalfSpeed() {
    // Drive at half speed when the right bumper is held
    m_driverController.rightBumper()
      .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
      .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
  }

  private void bind_PathWeaver() {
    m_driverController.x()
      .onTrue(m_robotDrive.getPathWeaverCommand());
  }

  /* ***************** CO-Driver Contols ************ */
  private void bind_CoAutoBalance() {
    // Drive to autobalance on teetertotter when 'X' button is pressed on codriver
    // controller, 5 second timeout
    // FIXME: change AutoBalancePID second parameter to CommandXboxController
    // m_codriverController.x()
    //   .onTrue(new AutoBalancePID(m_robotDrive, m_codriverController));
  }

  private void bind_CoLightToggle() {
    // When codriver button is pressed, toggle the light
    m_codriverController.y()
      .onTrue(new InstantCommand(() -> m_limelight.toggleLED()));
  }

  private void bind_ArmPID() {
    // Maybe need to enable/disable this when running commands
    // that will utilize the ground intake? Or just ensure
    // That the Arm setPoint remains at starting config setpoint?

    // Basic PID button commands for Arm Rotation
    m_driverController.a()
      .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kStowedAngle)));
    m_driverController.b()
      .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kHighNodeAngle)));
    m_driverController.x()
      .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kMidNodeAngle)));
    m_driverController.y()
      .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kLowNodeAngle)));
  }

  private void bind_ArmAndTelescope() {
    // Combination PID commands for Arm rotate & extend/retract
    m_codriverController.a()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kStowedPosition)),
        // new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kStowedAngle))
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kStowedAngle))));

    m_codriverController.b()
      .onTrue(new SequentialCommandGroup(
        // new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kHighNodeAngle)),
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kHighNodePosition)),
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kHighNodeAngle))));

    m_codriverController.x()
      .onTrue(new SequentialCommandGroup(
        // new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kMidNodeAngle)),
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kMidNodeAngle)),
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kMidNodePosition))));

    m_codriverController.y()
      .onTrue(new SequentialCommandGroup(
        // new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kLowNodeAngle)),
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kLowNodeAngle))));
        // new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kLowNodePosition)));

    m_codriverController.rightBumper()
      .onTrue(new InstantCommand(() -> m_claw.toggle()));
  }

  private void bind_LedModeTest() {
    // Signal for a CUBE when held
    m_codriverController.a()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.CUBE, SignalLEDs.LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF, SignalLEDs.LedPreference.MAIN, false)));

    // Signal for a CONE when held
    m_codriverController.b()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.CONE, SignalLEDs.LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF, SignalLEDs.LedPreference.MAIN, false)));

    m_codriverController.x()
      .toggleOnTrue(new InstantCommand(() -> m_claw.grabOrReleaseCube()));

    m_codriverController.y()
      .toggleOnTrue(new InstantCommand(() -> m_claw.grabOrReleaseCone()));
  }

  private void bind_LedSubsystemTest() {
    m_driverController.a()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.CONE, SignalLEDs.LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF, SignalLEDs.LedPreference.MAIN, false)));
    m_driverController.b()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.CUBE, SignalLEDs.LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF, SignalLEDs.LedPreference.MAIN, false)));
    m_driverController.x()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.RED, SignalLEDs.LedPreference.MAIN, false)));
    m_driverController.y()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF, SignalLEDs.LedPreference.MAIN, true)));
    m_driverController.back()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.BLUE, SignalLEDs.LedPreference.MAIN, true)));
    m_driverController.start()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.KITT, SignalLEDs.LedPreference.MAIN, true)));

    m_driverController.leftBumper()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.ALLIANCE, SignalLEDs.LedPreference.MAIN, true)));
    m_driverController.leftTrigger()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.GREEN, SignalLEDs.LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF, SignalLEDs.LedPreference.MAIN, false)));
    m_driverController.leftStick()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.COLONELS, SignalLEDs.LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF, SignalLEDs.LedPreference.MAIN, false)));
    m_driverController.rightBumper()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.COLONELS, SignalLEDs.LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF, SignalLEDs.LedPreference.MAIN, false)));
    m_driverController.rightTrigger()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.COUNTDOWN, SignalLEDs.LedPreference.MAIN, true)));
    m_driverController.rightStick()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.CONE, SignalLEDs.LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF, SignalLEDs.LedPreference.MAIN, false)));
  }

  // Return the command to run in autonomous
  public Command getAutonomousCommand() {
      return m_AutoChooser.getSelected();
  }
}
