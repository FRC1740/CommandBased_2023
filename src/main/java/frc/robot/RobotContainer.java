// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.wpilibj.XboxController.Button;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoBalancePID;
import frc.robot.commands.DriveToDistancePID;
//import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.DriveToDistance;
import frc.robot.subsystems.Manipulator.LedMode;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAngleProfiled;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final Manipulator m_manipulator = new Manipulator();
  private final Command m_autoCommand = new DriveToDistance(120, m_robotDrive);

  // The driver's controller
  private final XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);
  private final XboxController m_codriverController = new XboxController(Constants.OIConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // "Mario-Cart" drive: Triggers are gas and brake. Right stick turns left/right
        // Triggers are Axis 2; RightStick X is axis 3
        // Note the constants defined in the wpi XboxController class DO NOT MATCH the DS axes
        new RunCommand(() ->
            m_robotDrive.arcadeDrive(m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis(),
                  m_driverController.getLeftX(), true), m_robotDrive));
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
        .onTrue(new TurnToAngleProfiled(-90, m_robotDrive).withTimeout(5));

    // Auto-drive distance
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new DriveToDistancePID(50, m_robotDrive));

    // Manually rsest the gyro
    new JoystickButton(m_driverController, Button.kStart.value)
      .onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));

    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));


    /* ***************** CO-Driver Contols ************ */

    //Drive to autobalance on teetertotter when 'X' button is pressed on codriver controller, 5 second timeout
    new JoystickButton(m_codriverController, Button.kX.value)
        .onTrue(new AutoBalancePID(m_robotDrive));

    // When codriver button is pressed, toggle the light
    new JoystickButton(m_codriverController, Button.kY.value)
    .onTrue(new InstantCommand(()-> m_exampleSubsystem.toggle()));

    // Signal for a CUBE when held
    // new JoystickButton(m_codriverController, Button.kA.value)
    //     .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.mode.CUBE)))
    //     .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.mode.OFF)));
    
    // // Signal for a CONE when held
    // new JoystickButton(m_codriverController, Button.kB.value)
    //     .onTrue(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.mode.CONE)))
    //     .onFalse(new InstantCommand(() -> m_signalLEDs.setMode(SignalLEDs.mode.OFF)));
    
    // Solenoid test
    new JoystickButton(m_codriverController, Button.kA.value)
        .onTrue(new InstantCommand(() -> m_manipulator.Extend(LedMode.CUBE)))
        .onFalse(new InstantCommand(() -> m_manipulator.Retract()));

    // Signal for a CONE when held
    new JoystickButton(m_codriverController, Button.kB.value)
      .onTrue(new InstantCommand(() -> m_manipulator.Extend(LedMode.CONE)))
      .onFalse(new InstantCommand(() -> m_manipulator.Retract()));
        
    /* Signaling may be linked to the specific game piece intake
     * When we deploy the cone intake, signal "Yellow-orange"
     * When we deploy the cube intake, signal "purple"
     * 
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
