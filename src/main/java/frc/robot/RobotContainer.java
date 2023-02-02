// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.io.IOException;


import static edu.wpi.first.wpilibj.XboxController.Button;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoBalancePID;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.SequentialVisionAlign;
import frc.robot.commands.DriveOnAndBalanceChargeStation;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.commands.AprilTagAlign;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  protected final Claw m_Claw = new Claw();
  protected final Arm m_Arm = new Arm();

  // The driver's controller
  private final XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);
  private final XboxController m_codriverController = new XboxController(Constants.OIConstants.kCoDriverControllerPort);
  private final CommandXboxController m_codriverCommand = new CommandXboxController(Constants.OIConstants.kCoDriverControllerPort);
  
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  // auto command
  private final Command m_autoCommand = followPath("deploy/pathplanner/generatedJSON/Short_Straight_Path.wpilib.json", true);

  
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

    m_AutoChooser.addOption("curvy path", followPath("deploy/pathplanner/generatedJSON/Curvy Path.wpilib.json", true));
    
    m_AutoChooser.addOption("straight", followPath("deploy/pathplanner/generatedJSON/Straight path.wpilib.json", true));
    
    m_AutoChooser.addOption("Short Straight path", followPath("deploy/pathplanner/generatedJSON/Short_Straight_Path.wpilib.json",true));
    Shuffleboard.getTab("Autonomous").add(m_AutoChooser);
  }

  public Command followPath(String filename, boolean resetOdometry) {
    Trajectory trajectory;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) {
      DriverStation.reportError("Unable to open trajectory " + filename, exception.getStackTrace());
      System.out.println("Unable to read from file " + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
      m_robotDrive::getPose,
      new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
      DriveConstants.kDriveKinematics,
      m_robotDrive::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      m_robotDrive::tankDriveVolts,
      m_robotDrive);

    if (resetOdometry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    } else {
      return ramseteCommand;
    }

    m_Arm.setDefaultCommand( new RunCommand(() ->
          m_Arm.Rotate(m_codriverController.getRightY()*.1), m_Arm));

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
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new TurnToAngleProfiled(90, m_robotDrive).withTimeout(5));

    // Auto-drive distance
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new DriveToDistance(-240, m_robotDrive));

    // Manually rsest the gyro
    new JoystickButton(m_driverController, Button.kStart.value)
      .onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));

    new JoystickButton(m_driverController, Button.kY.value)
      .onTrue(new InstantCommand(() -> m_limelight.enableVisionProcessing()));


    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));


        new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(new AprilTagAlign(m_robotDrive, m_photonVision));
        
    /* ***************** CO-Driver Contols ************ */

    //Drive to autobalance on teetertotter when 'X' button is pressed on codriver controller, 5 second timeout
    new JoystickButton(m_codriverController, Button.kX.value)
        .onTrue(new AutoBalancePID(m_robotDrive, m_codriverController));

    // When codriver button is pressed, toggle the light
    new JoystickButton(m_codriverController, Button.kY.value)
    .onTrue(new InstantCommand(()-> m_limelight.toggleLED()));

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
    return m_autoCommand;
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
//         .onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.RED,SignalLEDs.LedPreference.MAIN,false)))
//         .onFalse(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF,SignalLEDs.LedPreference.MAIN,false)));
//       m_driverController.y()
//         .onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.OFF,SignalLEDs.LedPreference.MAIN,true)));
//       m_driverController.back().onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.BLUE,SignalLEDs.LedPreference.MAIN,true)));
//       m_driverController.start().onTrue(new InstantCommand(()-> m_signalLEDs.setMode(SignalLEDs.LedMode.KITT,SignalLEDs.LedPreference.MAIN,true)));
//     }
  
//   public Command getAutonomousCommand() {
//     return m_autoCommand;
//   }
// }
