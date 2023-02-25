// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import frc.board.RobotTab;
import frc.constants.ArmConstants;
import frc.constants.ClawConstants;
import frc.constants.DriveConstants;
import frc.constants.OIConstants;
import frc.constants.OIConstants.GamePiece;
import frc.robot.commands.AutoBalancePID;
import frc.robot.commands.DriveToDistance;
//import frc.robot.commands.SequentialVisionAlign;
//import frc.robot.commands.DriveOnAndBalanceChargeStation;
//import frc.robot.commands.RotateArmToAngle;
//import frc.robot.commands.AprilTagAlign;
import frc.robot.commands.TurnToAngleProfiled;

import frc.robot.subsystems.LimeLightSubsystem;
//import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
// import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.TelescopePIDSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.SignalLEDSubsystem;
import frc.robot.subsystems.SignalLEDSubsystem.LedMode;
import frc.robot.subsystems.SignalLEDSubsystem.LedPreference;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LimeLightSubsystem m_limelight = new LimeLightSubsystem();
  //private final PhotonVisionSubsystem m_photonVision = new PhotonVisionSubsystem();
  protected final ClawSubsystem m_claw = new ClawSubsystem();
  // protected final ArmPIDSubsystem m_arm = new ArmPIDSubsystem();
  protected final ArmProfiledPIDSubsystem m_armProfiled = new ArmProfiledPIDSubsystem();
  protected final TelescopePIDSubsystem m_telescope = new TelescopePIDSubsystem();
  protected final GroundIntakeSubsystem m_groundIntake = new GroundIntakeSubsystem();
  private final SignalLEDSubsystem m_signalLEDs = new SignalLEDSubsystem();

  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_codriverController = new CommandXboxController(OIConstants.kCoDriverControllerPort);
  
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  // auto command
  // Note: The autocommand is dynamically retrieved in getAutonomousCommand.
  // This member variable is currently unused
  // private Command m_autoCommand = m_AutoChooser.getSelected();

  private OIConstants.GamePiece m_gamePiece = OIConstants.kDefaultGamePiece;
  private RobotTab m_RobotTab;

  
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

    m_RobotTab = RobotTab.getInstance();
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    bind_POVTest();
    // bind_CircleTest();
    // bind_AutoDriveDistanceTest();
    // bind_ManualArmTest();
    // bind_ResetGyro();
    // bind_GroundIntakeTest();
    // bind_Limelight();
    // bind_HalfSpeed();
    // bind_PathWeaver();
    // bind_CoAutoBalance();
    // bind_CoLightToggle();
    // bind_CoCubeOp();

    // Enable the Arm PID Subsystem
    // m_arm.enable();
    m_armProfiled.enable();
    m_telescope.enable();

    // bind_ArmPIDTest();
    // bind_ArmAndTelescope();
    // bind_LedModeTest();
    // bind_LedSubsystemTest();

    bind_RC_ManualArm();
    bind_RC_AutoArm();
    bind_RC_GamePiece();
    bind_RC_RearIntake();
  }

  // See the Robot Control documents for the spec
  private void bind_RC_ManualArm() {
    // ManualArmUpDown
    // ManualArmExtendRetract
    m_codriverController.rightStick()
      .onTrue(new ParallelCommandGroup(
        new InstantCommand(() -> m_armProfiled.manualArmRotate(m_codriverController.getRightX())),
        new InstantCommand(() -> m_telescope.manualTelescope(m_codriverController.getRightY()))))
      .onFalse(new ParallelCommandGroup(
        new InstantCommand(() -> m_armProfiled.manualDone()),
        new InstantCommand(() -> m_telescope.manualDone())));

    // ManualRollerOut
    m_codriverController.rightTrigger()
      .onTrue(new InstantCommand(() -> m_claw.setIntakeSpeed(ClawConstants.EjectCubeManualSpeed)))
      .onFalse(new InstantCommand(() -> m_claw.setIntakeSpeed(0.0)));

    // ManualRollerIn
    m_codriverController.rightTrigger()
      .onTrue(new InstantCommand(() -> m_claw.setIntakeSpeed(ClawConstants.InjectCubeManualSpeed)))
      .onFalse(new InstantCommand(() -> m_claw.setIntakeSpeed(0.0)));

    // AllStow  
    m_codriverController.a()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kStowedPosition)),
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kStowedAngle))));

    // ManualClawOpen
    m_codriverController.rightBumper()
      .onTrue(new InstantCommand(() -> m_claw.open()));

    // ManualClawClose
    m_codriverController.leftBumper()
      .onTrue(new InstantCommand(() -> m_claw.close()));

    // ArmScore 
    m_driverController.x()
      .onTrue(new InstantCommand(() -> m_claw.score()))
      .onFalse(new InstantCommand(() -> m_claw.scoreDone()));
  }

  private void bind_RC_AutoArm() {
    // AutoArmScoreHigh
    m_codriverController.y()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kHighNodePosition)),
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kHighNodeAngle)),
        new InstantCommand(() -> m_claw.hold())
        ))
      .onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kStowedPosition)),
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kStowedAngle)),
        new InstantCommand(() -> m_claw.hold())
        ));
  
    // AutoArmScoreMedium
    m_codriverController.x()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kMidNodeAngle)),
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kMidNodePosition)),
        new InstantCommand(() -> m_claw.hold())
        ))
      .onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kStowedPosition)),
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kStowedAngle)),
        new InstantCommand(() -> m_claw.hold())
        ));

    // AutoArmScoreLow
    m_codriverController.start()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kLowNodeAngle)),
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kLowNodePosition)),
        new InstantCommand(() -> m_claw.hold())
        ))
      .onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kStowedPosition)),
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kStowedAngle)),
        new InstantCommand(() -> m_claw.hold())
        ));

    // AutoArmRetrieveMedium
    m_codriverController.b()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kMidRetrieveAngle)),
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kMidRetrievePosition)),
        new InstantCommand(() -> m_claw.retrieve())
        ))
      .onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kStowedPosition)),
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kStowedAngle)),
        new InstantCommand(() -> m_claw.hold())
        ));

    // AutoArmRetrieveLow
    m_codriverController.back()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kLowRetrieveAngle)),
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kLowRetrievePosition)),
        new InstantCommand(() -> m_claw.retrieve())
        ))
      .onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kStowedPosition)),
        new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kStowedAngle)),
        new InstantCommand(() -> m_claw.hold())
        ));
  }

  private void bind_RC_GamePiece() {
    // GamePieceToggle
    m_codriverController.leftStick()
      .onTrue(new InstantCommand(() -> toggleGamePiece()));
  }

  private void bind_RC_RearIntake() {
    // IntakeRetrieve
    m_driverController.b()
      // Operator must also stow the arm first!!
      .onTrue(new InstantCommand(() -> m_groundIntake.deploy()))
      .onFalse(new InstantCommand(() -> m_groundIntake.stow()));

    // IntakeGrasp
    m_driverController.a()
      .onTrue(new InstantCommand(() -> m_groundIntake.grasp()))
      .onFalse(new InstantCommand(() -> m_groundIntake.stopIntake()));

    // IntakeScore
    m_driverController.y()
      .onTrue(new InstantCommand(() -> m_groundIntake.eject()));
  }

  private void bind_POVTest() {
    // To use the D-Pad need to create a POVButton
    // There are 8 different possible D-Pad buttons based on the angle
    // 0 = up, 45 = up right, 90 = right ... 315 = up left
    // 
    // POVButton dPadUp = new POVButton(m_driverController.getHID(), 0);
    // POVButton dPadUpRight = new POVButton(m_driverController.getHID(), 45);
    // POVButton dPadRight = new POVButton(m_driverController.getHID(), 90);
    // POVButton dPadDownRight = new POVButton(m_driverController.getHID(), 135);
    // POVButton dPadDown = new POVButton(m_driverController.getHID(), 180);
    // POVButton dPadDownLeft = new POVButton(m_driverController.getHID(), 225);
    // POVButton dPadLeft = new POVButton(m_driverController.getHID(), 270);
    // POVButton dPadUpLeft = new POVButton(m_driverController.getHID(), 315);
    
    // D-Pad down button temporarily used to switch GamePiece Mode
    new POVButton(m_driverController.getHID(), 180)
      .onTrue(new InstantCommand(() -> toggleGamePiece()));
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
      .onTrue(new InstantCommand(() -> m_armProfiled.manualArmRotate(ArmConstants.kArmRotateManualSpeed)))
      .onFalse(new InstantCommand(() -> m_armProfiled.manualDone()));

    m_driverController.b()
      .onTrue(new InstantCommand(() -> m_armProfiled.manualArmRotate(-ArmConstants.kArmRotateManualSpeed)))
      .onFalse(new InstantCommand(() -> m_armProfiled.manualDone()));

    m_driverController.x()
      .onTrue(new InstantCommand(() -> m_telescope.manualTelescope(ArmConstants.kArmExtendManualSpeed)))
      .onFalse(new InstantCommand(() -> m_telescope.manualDone()));

    m_driverController.y()
      .onTrue(new InstantCommand(() -> m_telescope.manualTelescope(-ArmConstants.kArmExtendManualSpeed)))
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
    m_codriverController.back()
      .onTrue(new AutoBalancePID(m_robotDrive));
  }

  private void bind_CoLightToggle() {
    // When codriver button is pressed, toggle the light
    m_codriverController.y()
      .onTrue(new InstantCommand(() -> m_limelight.toggleLED()));
  }

  private void bind_CoCubeOp(){
    m_codriverController.leftBumper()
    .onTrue(new InstantCommand(() -> m_claw.intakeCube()))
    .onFalse(new InstantCommand(() -> m_claw.setClawSpeed(0)));

    m_codriverController.start()
    .onTrue(new InstantCommand(() -> m_claw.ejectCube()));
  }

  private void bind_ArmPIDTest() {
    // Maybe need to enable/disable this when running commands
    // that will utilize the ground intake? Or just ensure
    // That the Arm setPoint remains at starting config setpoint?

    // Basic PID button commands for Arm Rotation
    // m_driverController.a()
    //   .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kStowedAngle)));
    // m_driverController.b()
    //   .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kHighNodeAngle)));
    // m_driverController.x()
    //   .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kMidNodeAngle)));
    // m_driverController.y()
    //   .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kLowNodeAngle)));
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
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.CUBE, LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.OFF, LedPreference.MAIN, false)));

    // Signal for a CONE when held
    m_codriverController.b()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.CONE, LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.OFF, LedPreference.MAIN, false)));

    m_codriverController.x()
      .toggleOnTrue(new InstantCommand(() -> m_claw.grabOrReleaseCube()));

    m_codriverController.y()
      .toggleOnTrue(new InstantCommand(() -> m_claw.grabOrReleaseCone()));
  }

  private void bind_LedSubsystemTest() {
    m_driverController.a()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.CONE, LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.OFF, LedPreference.MAIN, false)));
    m_driverController.b()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.CUBE, LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.OFF, LedPreference.MAIN, false)));
    m_driverController.x()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.RED, LedPreference.MAIN, false)));
    m_driverController.y()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.OFF, LedPreference.MAIN, true)));
    m_driverController.back()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.BLUE, LedPreference.MAIN, true)));
    m_driverController.start()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.KITT, LedPreference.MAIN, true)));

    m_driverController.leftBumper()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.ALLIANCE, LedPreference.MAIN, true)));
    m_driverController.leftTrigger()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.GREEN, LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.OFF, LedPreference.MAIN, false)));
    m_driverController.leftStick()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.COLONELS, LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.OFF, LedPreference.MAIN, false)));
    m_driverController.rightBumper()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.COLONELS, LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.OFF, LedPreference.MAIN, false)));
    m_driverController.rightTrigger()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.COUNTDOWN, LedPreference.MAIN, true)));
    m_driverController.rightStick()
      .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.CONE, LedPreference.MAIN, false)))
      .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(LedMode.OFF, LedPreference.MAIN, false)));
  }

  // Return the command to run in autonomous
  public Command getAutonomousCommand() {
      return m_AutoChooser.getSelected();
  }

  public void autonomousInit() {
    if (DriverStation.isFMSAttached()) {
      m_robotDrive.burnFlash();
      m_claw.burnFlash();
      // m_arm.burnFlash();
      m_armProfiled.burnFlash();
      m_telescope.burnFlash();
      m_groundIntake.burnFlash();
    }
  }

  private void toggleGamePiece() {
    GamePiece newGamePiece = m_RobotTab.toggleGamePiece();
    setGamePiece(newGamePiece);
  }

  public void setGamePiece(OIConstants.GamePiece piece) {
    m_gamePiece = piece;
    if (m_gamePiece == OIConstants.GamePiece.CUBE) {
      m_signalLEDs.setMode(LedMode.CUBE, LedPreference.MAIN, false);
    } else if (m_gamePiece == OIConstants.GamePiece.CONE) {
      m_signalLEDs.setMode(LedMode.CONE, LedPreference.MAIN, false);
    }
    m_groundIntake.setGamePiece(piece);
    m_claw.setGamePiece(piece);
  }

}
