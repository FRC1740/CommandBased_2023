// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.board.ArmTab;
import frc.board.AutonomousTab;
import frc.board.ClawTab;
import frc.board.CommandTab;
import frc.board.DriveTrainTab;
import frc.board.GroundIntakeTab;
import frc.board.RobotTab;
import frc.board.VisionTab;
import frc.constants.ArmConstants;
import frc.constants.ClawConstants;
import frc.constants.DriveConstants;
import frc.constants.GroundIntakeConstants;
import frc.constants.OIConstants;
import frc.constants.OIConstants.GamePiece;
import frc.robot.commands.AutoBalancePID;
import frc.robot.commands.auto.*;
import frc.robot.commands.driver.ArmStow;
import frc.robot.commands.driver.AutoArmScoreHigh;
import frc.robot.commands.driver.AutoArmScoreLow;
import frc.robot.commands.driver.AutoArmScoreMedium;
import frc.robot.commands.driver.DunkScore;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.DriveToDistanceVision;
import frc.robot.commands.LimelightAssistedDrive;
import frc.robot.commands.SequentialVisionAlign;
import frc.robot.commands.TurnToAngle;
// import frc.robot.commands.SubStationSideAuto;
//import frc.robot.commands.SequentialVisionAlign;
//import frc.robot.commands.DriveOnAndBalanceChargeStation;
//import frc.robot.commands.RotateArmToAngle;
//import frc.robot.commands.AprilTagAlign;
import frc.robot.commands.TurnToAngleProfiled;

import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.math.util.Units;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // New ------------------------------------------------------------------------------------------
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem m_robotDrive;
  private LimeLightSubsystem m_limelight;
  private PhotonVisionSubsystem m_photonVision;
  protected ClawSubsystem m_claw;
  // protected final ArmPIDSubsystem m_arm;
  protected ArmProfiledPIDSubsystem m_armProfiled;
  protected TelescopePIDSubsystem m_telescope;
  protected GroundIntakeSubsystem m_groundIntake;
  private SignalLEDSubsystem m_signalLEDs;

  private Paths m_paths;

  private CommandXboxController m_driverController;
  private CommandXboxController m_codriverController;

  private RobotShared m_robotShared;
  // Old ------------------------------------------------------------------------------------------
  // // The robot's subsystems and commands are defined here...
  // private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private final LimeLightSubsystem m_limelight = new LimeLightSubsystem();
  // //private final PhotonVisionSubsystem m_photonVision = new PhotonVisionSubsystem();
  // protected final ClawSubsystem m_claw = new ClawSubsystem();
  // // protected final ArmPIDSubsystem m_arm = new ArmPIDSubsystem();
  // protected final ArmProfiledPIDSubsystem m_armProfiled = new ArmProfiledPIDSubsystem();
  // protected final TelescopePIDSubsystem m_telescope = new TelescopePIDSubsystem();
  // protected final GroundIntakeSubsystem m_groundIntake = new GroundIntakeSubsystem();
  // private final SignalLEDSubsystem m_signalLEDs = new SignalLEDSubsystem();

  // private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  // private final CommandXboxController m_codriverController = new CommandXboxController(OIConstants.kCoDriverControllerPort);
  // ----------------------------------------------------------------------------------------------
  
  private OIConstants.GamePiece m_gamePiece = OIConstants.kDefaultGamePiece;
  private RobotTab m_RobotTab;
  private AutonomousTab m_AutonomousTab;

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initSubsystems();
    initShuffleboard();

    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drivem_robotDrive
    m_robotDrive.setDefaultCommand(
      // "Mario-Cart" drive: Triggers are gas and brake. Right stick turns left/right
      // Triggers are Axis 2; RightStick X is axis 3
      // Note the constants defined in the wpi XboxController class DO NOT MATCH the DS axes
      new RunCommand(() ->
        m_robotDrive.arcadeDrive(m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis(),
        m_driverController.getLeftX(), false), m_robotDrive));
    
    // Set the starting game piece
    m_AutonomousTab.setGamePiece(m_RobotTab.getGamePiece());
  }

  private void initSubsystems() {
    // New ------------------------------------------------------------------------------------------
    m_robotShared = RobotShared.getInstance();

    m_paths = m_robotShared.getPaths();
    m_robotDrive = m_robotShared.getDriveSubsystem();
    m_limelight = m_robotShared.getLimeLightSubsystem();
    m_photonVision = m_robotShared.getPhotonVisionSubsystem();
    m_claw = m_robotShared.getClawSubsystem();
    m_armProfiled = m_robotShared.getArmProfiledPIDSubsystem();
    m_telescope = m_robotShared.getTelescopePIDSubsystem();
    m_groundIntake = m_robotShared.getGroundIntakeSubsystem();
    m_signalLEDs = m_robotShared.getSignalLEDSubsystem();
    m_driverController = m_robotShared.getDriverController();
    m_codriverController = m_robotShared.getCodriverController();
    // ----------------------------------------------------------------------------------------------
  }

  private void initShuffleboard() {
    m_RobotTab = RobotTab.getInstance();
    ArmTab.getInstance();
    m_AutonomousTab = AutonomousTab.getInstance();
    ClawTab.getInstance();
    DriveTrainTab.getInstance();
    GroundIntakeTab.getInstance();
    VisionTab.getInstance();
    CommandTab.getInstance();
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    // bind_POVTest();
    // bind_CircleTest();
    // bind_AutoDriveDistanceTest();
    // bind_ManualArmTest();
    // bind_ResetGyro();
    // bind_GroundIntakeTest();
    bind_Limelight();
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
    bind_autoBalance();

    bind_constant_speed_drive();

    bind_RC_ManualArm();
    bind_RC_AutoArm();
    bind_RC_GamePiece();
    bind_RC_RearIntake();

    bind_Auto_Tests();
  }

  // expected to be temporary
  private void bind_Auto_Tests() {
    // Currently none of these are in use

    new POVButton(m_driverController.getHID(), 0)
      .whileTrue(new RB_2_Exit_Turn_Balance_Vision());

    new POVButton(m_driverController.getHID(), 90)
      .whileTrue(new RB_2_Cube_Balance());

    // new POVButton(m_driverController.getHID(), 180)
    //   .onTrue(new RB_2_Pickup());

    // new POVButton(m_driverController.getHID(), 270)
    //   .onTrue(new RB_2_Exit_Balance());

    // m_driverController.leftBumper()
    //   .onTrue(new RB_3());
  

    m_driverController.x()
      //.whileTrue(m_robotDrive.FollowPathWithEvents(m_paths.Blue_3_McTriple, true));
      .whileTrue(new Blue_3_McDouble_Deluxe());

    m_driverController.rightStick()
      .whileTrue(new RB_2_Exit_Balance_Vision());

      }

  private void bind_constant_speed_drive(){
    m_driverController.rightBumper()
    .whileTrue(new RunCommand(() -> m_robotDrive.arcadeDrive(DriveConstants.kConstantSpeedDrive, m_driverController.getLeftX(), false), m_robotDrive));

    m_driverController.back()
    .whileTrue(new RunCommand(() -> m_robotDrive.arcadeDrive(-DriveConstants.kConstantSpeedDrive, m_driverController.getLeftX(), false), m_robotDrive));
  }
  // See the Robot Control documents for the spec
  private void bind_RC_ManualArm() {
    // ManualArmUpDown
    m_codriverController.leftStick()
      .whileTrue(new RunCommand(() -> m_armProfiled.manualArmRotate(m_codriverController.getLeftY())))
      .onFalse(new InstantCommand(() -> m_armProfiled.manualDone()));

    // ManualArmExtendRetract
    m_codriverController.rightStick()
    .whileTrue(new RunCommand(() -> m_telescope.manualTelescope(m_codriverController.getRightY())))
    .onFalse(new InstantCommand(() -> m_telescope.manualDone()));

    // ManualRollerOut
    new POVButton(m_codriverController.getHID(), 0)
    // m_codriverController.rightTrigger()
      .whileTrue(new RunCommand(() -> m_claw.setIntakeSpeed(ClawConstants.kManualEjectSpeed)))
      .onFalse(new InstantCommand(() -> m_claw.setIntakeSpeed(0.0)));

    // ManualRollerIn
    new POVButton(m_codriverController.getHID(), 180)
    // m_codriverController.rightTrigger()
      .whileTrue(new RunCommand(() -> m_claw.setIntakeSpeed(ClawConstants.kManualInjectSpeed)))
      .onFalse(new InstantCommand(() -> m_claw.setIntakeSpeed(0.0)));

    // AllStow  
    m_codriverController.a()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.STOWED))),
        new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.STOWED)))));

    // ManualClawOpen
    new POVButton(m_codriverController.getHID(), 90)
      .onTrue(new InstantCommand(() -> m_claw.open()));

    // ManualClawClose
    new POVButton(m_codriverController.getHID(), 270)
      .onTrue(new InstantCommand(() -> m_claw.close()));

    // ArmScore 
    m_codriverController.leftTrigger()
      .whileTrue(
        new RunCommand(() -> m_claw.score()))
        // new DunkScore())
      .onFalse(new InstantCommand(() -> m_claw.scoreDone()));
  }

  private void bind_RC_AutoArm() {
    // AutoArmScoreHigh
    m_codriverController.x()
      .onTrue(new AutoArmScoreHigh()
        // new SequentialCommandGroup(
        // new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.HIGH))),
        // new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.HIGH))),
        // new InstantCommand(() -> m_claw.hold())
        // )
        )
      .onFalse(new ArmStow()
        //  new SequentialCommandGroup(
        //   new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.STOWED))),
        //   new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.STOWED)))
        // )
        );
  
    // AutoArmScoreMedium
    m_codriverController.y()
      .onTrue(new AutoArmScoreMedium()
        // new SequentialCommandGroup(
        // new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.MID))),
        // new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.MID))),
        // new InstantCommand(() -> m_claw.hold())
        // )
        )
      .onFalse(new ArmStow()
        // new SequentialCommandGroup(
        // new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.STOWED))),
        // new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.STOWED)))
        // )
        );

    // AutoArmScoreLow
    m_codriverController.b()
      .onTrue(new AutoArmScoreLow()
        // new SequentialCommandGroup(
        // new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.LOW))),
        // new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.LOW))),
        // new InstantCommand(() -> m_claw.hold())
        // )
        )
      .onFalse(new ArmStow()
        // new SequentialCommandGroup(
        // new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.STOWED))),
        // new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.STOWED)))
        // )
        );

    // AutoArmRetrieveMedium
    m_codriverController.rightBumper()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.SHELF))),
        new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.SHELF))),
        new InstantCommand(() -> m_claw.retrieve())
        ))
      .onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> m_claw.hold()),
        new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.STOWED))),
        new WaitCommand(0.3),
        new InstantCommand(() -> m_claw.setClawSpeed(0)),
        new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.STOWED)))
        
        ));

    // AutoArmRetrieveLow
    m_codriverController.rightTrigger()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.FLOOR))),
        new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.FLOOR))),
        new InstantCommand(() -> m_claw.retrieve())
        ))
      .onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> m_claw.hold()), 
        new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.STOWED))),
        new WaitCommand(0.3),
        new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.STOWED))),
        new InstantCommand(() -> m_claw.setClawSpeed(0))
        ));
  }

  private void bind_RC_GamePiece() {
    // Driver
    // m_driverController.back()
    //   .onTrue(new InstantCommand(() -> setGamePieceCone()));

    // m_driverController.start()
    //   .onTrue(new InstantCommand(() -> setGamePieceCube()));

    // Codriver
    m_codriverController.back()
      .onTrue(new InstantCommand(() -> setGamePieceCone()));

    m_codriverController.start()
      .onTrue(new InstantCommand(() -> setGamePieceCube()));

  }

  private void bind_RC_RearIntake() {
    // IntakeRetrieve
    m_driverController.a()
      // Operator must also stow the arm first!!
      .onTrue(new InstantCommand(() -> m_groundIntake.deploy(GroundIntakeConstants.kCubeIntakeSpeed)))
      .onFalse(new InstantCommand(() -> m_groundIntake.stow()));

    // IntakeGrasp
    m_driverController.y()
      .onTrue(new InstantCommand(() -> m_groundIntake.eject(GroundIntakeConstants.kCubeEjectSpeedLow)))
      .onFalse(new InstantCommand(() -> m_groundIntake.stopIntake()));
      // .onTrue(new RB_2_Cube_Balance(OIConstants.GamePiece.CUBE));

    // IntakeScore
    m_driverController.b()
      .whileTrue(new RunCommand(() -> m_groundIntake.eject(GroundIntakeConstants.kCubeEjectSpeed)))
      .onFalse(new InstantCommand(() -> m_groundIntake.stopIntake()));
  }

  private void bind_autoBalance(){
    m_driverController.start()
      .whileTrue(new AutoBalancePID(m_robotDrive));
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
      .onTrue(new InstantCommand(() -> m_groundIntake.deploy(GroundIntakeConstants.kCubeIntakeSpeed)))
      .onFalse(new InstantCommand(() -> m_groundIntake.stow()));

    m_driverController.leftBumper()
      .onTrue(new InstantCommand(() -> m_groundIntake.eject(GroundIntakeConstants.kCubeEjectSpeed)))
      .onFalse(new InstantCommand(() -> m_groundIntake.stow()));
  }

  private void bind_Limelight() {
    m_driverController.leftBumper()
      .whileTrue(new LimelightAssistedDrive());
  }

  private void bind_HalfSpeed() {
    // Drive at half speed when the right bumper is held
    m_driverController.rightBumper()
      .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
      .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
  }

  // private void bind_PathWeaver() {
  //   m_driverController.x()
  //     .onTrue(m_robotDrive.getPathWeaverCommand(true));
  // }

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
    //   .onTrue(new InstantCommand(() -> m_arm.setSetpoint(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.STOWED))));
    // m_driverController.b()
    //   .onTrue(new InstantCommand(() -> m_arm.setSetpoint(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.HIGH))));
    // m_driverController.x()
    //   .onTrue(new InstantCommand(() -> m_arm.setSetpoint(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.MID))));
    // m_driverController.y()
    //   .onTrue(new InstantCommand(() -> m_arm.setSetpoint(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.LOW))));
  }


  private void bind_ArmAndTelescope() {
    // Combination PID commands for Arm rotate & extend/retract
    m_codriverController.a()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.STOWED))),
        // new InstantCommand(() -> m_arm.setSetpoint(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.STOWED)))
        new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.STOWED)))));

    m_codriverController.b()
      .onTrue(new SequentialCommandGroup(
        // new InstantCommand(() -> m_arm.setSetpoint(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.HIGH))),
        new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.HIGH))),
        new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.HIGH)))));

    m_codriverController.x()
      .onTrue(new SequentialCommandGroup(
        // new InstantCommand(() -> m_arm.setSetpoint(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.MID))),
        new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.MID))),
        new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.MID)))));

    m_codriverController.y()
      .onTrue(new SequentialCommandGroup(
        // new InstantCommand(() -> m_arm.setSetpoint(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.LOW))),
        new InstantCommand(() -> m_armProfiled.setGoal(m_robotShared.calculateArmSetpoint(ArmConstants.AutoMode.LOW)))));
        // new InstantCommand(() -> m_telescope.setSetpoint(m_robotShared.calculateTeleSetpoint(ArmConstants.AutoMode.LOW))));

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
      Command autonomousCommand = m_AutonomousTab.getAutonomousCommand();
      if (autonomousCommand == null) {
        autonomousCommand = new RB_1();
      }
      return autonomousCommand;
  }

  public void disabledInit() {
    m_signalLEDs.setMode(LedMode.COLONELS, LedPreference.MAIN, true);
  }

  public void autonomousInit() {
    // if (DriverStation.isFMSAttached()) {
      m_robotDrive.burnFlash();
      m_claw.burnFlash();
      // m_arm.burnFlash();
      m_armProfiled.burnFlash();
      m_telescope.burnFlash();
      m_groundIntake.burnFlash();
    // }
    m_signalLEDs.setMode(LedMode.ALLIANCE, LedPreference.MAIN, true);
  }

  private void toggleGamePiece() {
    GamePiece newGamePiece = m_RobotTab.toggleGamePiece();
    setGamePiece(newGamePiece);
  }

  private void setGamePieceCone() {
    GamePiece newGamePiece = m_RobotTab.setGamePieceCone();
    setGamePiece(newGamePiece);
  }

  private void setGamePieceCube() {
    GamePiece newGamePiece = m_RobotTab.setGamePieceCube();
    setGamePiece(newGamePiece);
  }

  public void setGamePiece(OIConstants.GamePiece piece) {
    m_gamePiece = piece;
    if (m_gamePiece == OIConstants.GamePiece.CUBE) {
      m_signalLEDs.setMode(LedMode.CUBE, LedPreference.MAIN, true);
    } else if (m_gamePiece == OIConstants.GamePiece.CONE) {
      m_signalLEDs.setMode(LedMode.CONE, LedPreference.MAIN, true);
    }
    m_groundIntake.setGamePiece(piece);
    m_claw.setGamePiece(piece);
  }

}
