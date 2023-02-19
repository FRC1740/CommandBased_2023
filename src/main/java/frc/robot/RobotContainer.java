// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;



import static edu.wpi.first.wpilibj.XboxController.Button;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.commands.AutoBalancePID;
import frc.robot.commands.DriveToDistance;
//import frc.robot.commands.SequentialVisionAlign;
//import frc.robot.commands.DriveOnAndBalanceChargeStation;
//import frc.robot.commands.RotateArmToAngle;
//import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ProfiledPIDArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Claw; // Uncomment this when mechanism is ready to test
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.ArmExtensionPID;
import frc.robot.subsystems.GroundIntake;
import frc.constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TurnToAngleProfiled;
//import frc.robot.commands.AprilTagAlign;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import frc.constants.DriveConstants;
import frc.constants.OIConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private final PhotonVision m_photonVision = new PhotonVision();
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  protected final Claw m_Claw = new Claw();
  //protected final ArmPIDSubsystem m_arm = new ArmPIDSubsystem();
  protected final ProfiledPIDArmSubsystem m_ProfiledArm = new ProfiledPIDArmSubsystem();
  protected final ArmExtensionPID m_telescope = new ArmExtensionPID();
  protected final GroundIntake m_groundIntake = new GroundIntake();

  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_codriverController = new XboxController(OIConstants.kCoDriverControllerPort);
  // FIXME: We should be migrating away form the XboxController class to the CommandXboxController class
  // private final CommandXboxController m_codriverCommand = new CommandXboxController(Constants.OIConstants.kCoDriverControllerPort);
  
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  // auto command
  private Command m_autoCommand = m_AutoChooser.getSelected();

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
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
    new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared)), 
    true));
    
    m_AutoChooser.addOption("straight", m_robotDrive.FollowPath(PathPlanner.loadPath("Straight path", 
    new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared)), 
    true));
    
    m_AutoChooser.addOption("Short Straight path", m_robotDrive.FollowPath(PathPlanner.loadPath("Short_Straight_Path", 
    new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared)), 
    true));

    m_AutoChooser.addOption("more curvy path", m_robotDrive.FollowPath(PathPlanner.loadPath("more curvy path", 
    new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared)), 
    true));

    Shuffleboard.getTab("Autonomous").add(m_AutoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* ***************** Driver Contols ************ */

    // Turn to -90 degrees with a profile when the Circle button is pressed, with a 5 second timeout
    // new JoystickButton(m_driverController, Button.kA.value)
    //     .onTrue(new TurnToAngleProfiled(90, m_robotDrive).withTimeout(5));

    // Auto-drive distance
    // new JoystickButton(m_driverController, Button.kB.value)
    //     .onTrue(new DriveToDistance(5, m_robotDrive));
if (false) { 
    new JoystickButton(m_driverController, Button.kA.value)
      .onTrue(new InstantCommand(() -> m_ProfiledArm.manualArmRotateUp()))
      .onFalse(new InstantCommand(() -> m_ProfiledArm.manualDone()));

    new JoystickButton(m_driverController, Button.kB.value)
      .onTrue(new InstantCommand(() -> m_ProfiledArm.manualArmRotateDown()))
      .onFalse(new InstantCommand(() -> m_ProfiledArm.manualDone()));

    new JoystickButton(m_driverController, Button.kX.value)
      .onTrue(new InstantCommand(() -> m_telescope.manualTelescopeOut()))
      .onFalse(new InstantCommand(() -> m_telescope.manualDone()));

    new JoystickButton(m_driverController, Button.kY.value)
      .onTrue(new InstantCommand(() -> m_telescope.manualTelescopeIn()))
      .onFalse(new InstantCommand(() -> m_telescope.manualDone()));

    // Manually rsest the gyro
    new JoystickButton(m_driverController, Button.kStart.value)
      .onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));
}

if (true) { // GroundIntake testing
    new JoystickButton(m_driverController, Button.kRightBumper.value)
      // This command must also stow the arm first!!
      .onTrue(new InstantCommand(() -> m_groundIntake.deploy()))
      .onFalse(new InstantCommand(() -> m_groundIntake.stow()));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
      .onTrue(new InstantCommand(() -> m_groundIntake.eject()))
      .onFalse(new InstantCommand(() -> m_groundIntake.stow()));

    new JoystickButton(m_driverController, Button.kX.value)
      .onTrue(new InstantCommand(() -> m_groundIntake.setIntakeSpeed(0.1)));

    new JoystickButton(m_driverController, Button.kY.value)
      .onTrue(new InstantCommand(() -> m_groundIntake.setIntakeSpeed(-0.1)));
}

    // new JoystickButton(m_driverController, Button.kY.value)
    //   .onTrue(new InstantCommand(() -> m_limelight.enableVisionProcessing()));


    // // Drive at half speed when the right bumper is held
    // new JoystickButton(m_driverController, Button.kRightBumper.value)
    //     .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
    //     .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));


    // new JoystickButton(m_driverController, Button.kX.value)
    //     .onTrue(m_robotDrive.getPathWeaverCommand());
        
    /* ***************** CO-Driver Contols ************ */

    //Drive to autobalance on teetertotter when 'X' button is pressed on codriver controller, 5 second timeout
    // new JoystickButton(m_codriverController, Button.kX.value)
    //     .onTrue(new AutoBalancePID(m_robotDrive, m_codriverController));

    // // When codriver button is pressed, toggle the light
    // new JoystickButton(m_codriverController, Button.kY.value)
    // .onTrue(new InstantCommand(()-> m_limelight.toggleLED()));

    // Enable the Arm PID Subsystem
    // Maybe need to enable/disable this when running commands
    // that will utilize the ground intake? Or just ensure
    // That the Arm setPoint remains at starting config setpoint?

    //m_arm.enable();
    m_ProfiledArm.enable();
    m_telescope.enable();

    // Basic PID button commands for Arm Rotation
    // new JoystickButton(m_driverController, Button.kA.value)
    //     .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kStowedAngle)));
    // new JoystickButton(m_driverController, Button.kB.value)
    //     .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kHighNodeAngle)));
    // new JoystickButton(m_driverController, Button.kX.value)
    //     .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kMidNodeAngle)));
    // new JoystickButton(m_driverController, Button.kY.value)
    //     .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kLowNodeAngle)));

if (false) {
    // Combination PID commands for Arm rotate & extend/retract
    new JoystickButton(m_codriverController, Button.kA.value)
      .onTrue(new SequentialCommandGroup(
          new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kStowedPosition)),
          //new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kStowedAngle))
          new InstantCommand(() -> m_ProfiledArm.setGoal(ArmConstants.kStowedAngle))));

    new JoystickButton(m_codriverController, Button.kB.value)
      .onTrue(new SequentialCommandGroup(
          //new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kHighNodeAngle)),
          new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kHighNodePosition)),
          new InstantCommand(() -> m_ProfiledArm.setGoal(ArmConstants.kHighNodeAngle))));

    new JoystickButton(m_codriverController, Button.kX.value)
      .onTrue(new SequentialCommandGroup(
          //new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kMidNodeAngle)),
          new InstantCommand(() -> m_ProfiledArm.setGoal(ArmConstants.kMidNodeAngle)),
          new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kMidNodePosition))));
    
    new JoystickButton(m_codriverController, Button.kY.value)
      .onTrue(new SequentialCommandGroup(
          //new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kLowNodeAngle)),
          new InstantCommand(() -> m_ProfiledArm.setGoal(ArmConstants.kLowNodeAngle))));
          //new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kLowNodePosition))));

    new JoystickButton(m_codriverController, Button.kRightBumper.value)
      .onTrue(new InstantCommand(() -> m_Claw.toggle()));
}

      
      // Signal for a CUBE when held
    // new JoystickButton(m_codriverController, Button.kA.value)
    //     .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.mode.CUBE)))
    //     .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.mode.OFF)));
    
    // // Signal for a CONE when held
    // new JoystickButton(m_codriverController, Button.kB.value)
    //     .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.mode.CONE)))
    //     .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.mode.OFF)));
    
    /*
    new JoystickButton(m_codriverController, Button.kA.value)
      .toggleOnTrue(new InstantCommand(() -> m_Claw.GrabOrReleaseCube()));

    // Signal for a CONE when held
    new JoystickButton(m_codriverController, Button.kB.value)
      .toggleOnTrue(new InstantCommand(() -> m_Claw.GrabOrReleaseCone()));
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_AutoChooser.getSelected();
  }
}

//   package frc.robot;
//   import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//   import edu.wpi.first.wpilibj2.command.Command;
//   import edu.wpi.first.wpilibj2.command.InstantCommand;
//   import frc.robot.subsystems.SignalLEDs;
  
//   public class RobotContainer {
//     private final SignalLEDs m_signalLEDs = new SignalLEDs();
//     private final CommandXboxController m_driverController = new CommandXboxController(Constants.OIConstants.kDriverControllerPort);
//     private final Command m_autoCommand = null;
//     public RobotContainer() {
//       configureButtonBindings();
//     }
//     private void configureButtonBindings() {
//       m_driverController.a()
//         .onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.CONE,SignalLEDs.LedPreference.MAIN,false)))
//         .onFalse(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF,SignalLEDs.LedPreference.MAIN,false)));
//       m_driverController.b()
//         .onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.CUBE,SignalLEDs.LedPreference.MAIN,false)))
//         .onFalse(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF,SignalLEDs.LedPreference.MAIN,false)));
//       m_driverController.x()
//         .onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.RED,SignalLEDs.LedPreference.MAIN,false)));
//       m_driverController.y()
//         .onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF,SignalLEDs.LedPreference.MAIN,true)));
//       m_driverController.back().onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.BLUE,SignalLEDs.LedPreference.MAIN,true)));
//       m_driverController.start().onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.KITT,SignalLEDs.LedPreference.MAIN,true)));

//       m_driverController.leftBumper()
//         .onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.ALLIANCE,SignalLEDs.LedPreference.MAIN,true)));
//       m_driverController.leftTrigger()
//         .onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.GREEN,SignalLEDs.LedPreference.MAIN,false)))
//         .onFalse(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF,SignalLEDs.LedPreference.MAIN,false)));
//       m_driverController.leftStick()
//         .onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.COLONELS,SignalLEDs.LedPreference.MAIN,false)))
//         .onFalse(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF,SignalLEDs.LedPreference.MAIN,false)));
//       m_driverController.rightBumper()
//         .onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.COLONELS,SignalLEDs.LedPreference.MAIN,false)))
//         .onFalse(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF,SignalLEDs.LedPreference.MAIN,false)));
//       m_driverController.rightTrigger()
//         .onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.COUNTDOWN,SignalLEDs.LedPreference.MAIN,true)));
//       m_driverController.rightStick()
//         .onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.CONE,SignalLEDs.LedPreference.MAIN,false)))
//         .onFalse(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF,SignalLEDs.LedPreference.MAIN,false)));
//     }
  
//   public Command getAutonomousCommand() {
//     return m_autoCommand;
//   }
// }
