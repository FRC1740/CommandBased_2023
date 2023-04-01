// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.board.ArmTab;
import frc.constants.ArmConstants;
import frc.constants.GroundIntakeConstants;
import frc.constants.ArmConstants.AutoMode;
import frc.robot.commands.AutoBalancePID;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.commands.basic.ArmManual;
import frc.robot.commands.basic.ClawClose;
import frc.robot.commands.basic.ClawOpen;
import frc.robot.commands.basic.ClawRollerIn;
import frc.robot.commands.basic.ClawRollerOut;
import frc.robot.commands.basic.ClawScore;
import frc.robot.commands.basic.IntakeDeploy;
import frc.robot.commands.basic.IntakeEject;
import frc.robot.commands.basic.IntakeGrasp;
import frc.robot.commands.basic.IntakeStop;
import frc.robot.commands.basic.IntakeStow;
import frc.robot.commands.basic.TelescopeManual;
import frc.robot.commands.driver.AutoArmRetrieveLow;
import frc.robot.commands.driver.AutoArmRetrieveMedium;
import frc.robot.commands.driver.AutoArmScoreHigh;
import frc.robot.commands.driver.AutoArmScoreLow;
import frc.robot.commands.driver.AutoArmScoreMedium;
import frc.robot.commands.driver.ArmStow;
import frc.robot.commands.driver.ArmStowHold;
import frc.robot.commands.driver.ArmStowHoldDelay;
import frc.robot.commands.driver.ToggleGamePiece;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SignalLEDSubsystem;
import frc.robot.subsystems.SignalLEDSubsystem.LedMode;
import frc.robot.subsystems.SignalLEDSubsystem.LedPreference;
import frc.robot.subsystems.TelescopePIDSubsystem;

/** Add your docs here. */
public class ButtonBindings {

    private CommandXboxController m_driverController = null;
    private CommandXboxController m_codriverController = null;

    public ButtonBindings() {
        RobotShared robotShared = RobotShared.getInstance();
        m_driverController = robotShared.getDriverController();
        m_codriverController = robotShared.getCodriverController();

        bind_DefaultDriveCommand(m_driverController);

        bind_DriverRobotControl(m_driverController);
        bind_CodriverRobotControl(m_codriverController);
    }

    private void bind_DefaultDriveCommand(CommandXboxController controller) {
        RobotShared robotShared = RobotShared.getInstance();
        DriveSubsystem m_robotDrive = robotShared.getDriveSubsystem();

        // Configure default commands
        // Set the default drive command to split-stick arcade drivem_robotDrive
        m_robotDrive.setDefaultCommand(
          
        
            // "Mario-Cart" drive: Triggers are gas and brake. Right stick turns left/right
            // Triggers are Axis 2; RightStick X is axis 3
            // Note the constants defined in the wpi XboxController class DO NOT MATCH the DS axes
            new RunCommand(() ->
                m_robotDrive.arcadeDrive(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis(),
                controller.getLeftX(), true), m_robotDrive));

    }

    private void bind_DriverRobotControl(CommandXboxController controller) {
        Trigger driver_A = controller.a();
        Trigger driver_B = controller.b();
        Trigger driver_X = controller.x();
        Trigger driver_Y = controller.y();
        Trigger driver_LeftStick = controller.leftStick();
        Trigger driver_LeftTrigger = controller.leftTrigger();
        Trigger driver_LeftBumper = controller.leftBumper();
        Trigger driver_Start = controller.start();
        Trigger driver_Back = controller.back();
        Trigger driver_RightStick = controller.rightStick();
        Trigger driver_RightTrigger = controller.rightTrigger();
        Trigger driver_RightBumper = controller.rightBumper();
        Trigger driver_DPadUp = new POVButton(controller.getHID(), 0);
        Trigger driver_DPadRight = new POVButton(controller.getHID(), 90);
        Trigger driver_DPadDown = new POVButton(controller.getHID(), 180);
        Trigger driver_DPadLeft = new POVButton(controller.getHID(), 270);

        // IntakeRetrieve
        driver_B.onTrue(new IntakeDeploy(GroundIntakeConstants.kCubeIntakeSpeed))
            .onFalse(new IntakeStow());

        // IntakeGrasp
        driver_A.onTrue(new IntakeGrasp(GroundIntakeConstants.kCubeGraspSpeed))
            .onFalse(new IntakeStop());

        // IntakeScore
        driver_Y.whileTrue(new IntakeEject(GroundIntakeConstants.kCubeEjectSpeed))
            .onFalse(new IntakeStop());

        // GamePieceToggle
        driver_Start.onTrue(new ToggleGamePiece());
        
    }
    
    private void bind_CodriverRobotControl(CommandXboxController controller) {
        Trigger codriver_A = controller.a();
        Trigger codriver_B = controller.b();
        Trigger codriver_X = controller.x();
        Trigger codriver_Y = controller.y();
        Trigger codriver_LeftStick = controller.leftStick();
        Trigger codriver_LeftTrigger = controller.leftTrigger();
        Trigger codriver_LeftBumper = controller.leftBumper();
        Trigger codriver_Start = controller.start();
        Trigger codriver_Back = controller.back();
        Trigger codriver_RightStick = controller.rightStick();
        Trigger codriver_RightTrigger = controller.rightTrigger();
        Trigger codriver_RightBumper = controller.rightBumper();
        Trigger codriver_DPadUp = new POVButton(controller.getHID(), 0);
        Trigger codriver_DPadRight = new POVButton(controller.getHID(), 90);
        Trigger codriver_DPadDown = new POVButton(controller.getHID(), 180);
        Trigger codriver_DPadLeft = new POVButton(controller.getHID(), 270);

        // ManualArmUpDown
        codriver_LeftStick.whileTrue(new ArmManual(m_codriverController));
        
        // ManualArmExtendRetract
        codriver_RightStick.whileTrue(new TelescopeManual(m_codriverController));
        
        // ManualRollerOut
        codriver_DPadUp.whileTrue(new ClawRollerOut());

        // ManualRollerIn
        codriver_DPadDown.whileTrue(new ClawRollerIn());

        // AllStow  
        codriver_A.onTrue(new ArmStow());

        // ManualClawOpen
        codriver_DPadRight.onTrue(new ClawOpen());

        // ManualClawClose
        codriver_DPadLeft.onTrue(new ClawClose());

        // ArmScore 
        codriver_LeftTrigger.whileTrue(new ClawScore());

        // AutoArmScoreHigh
        codriver_X.onTrue(new AutoArmScoreHigh())
            .onFalse(new ArmStowHold());

        // AutoArmScoreMedium
        codriver_Y.onTrue(new AutoArmScoreMedium())
            .onFalse(new ArmStowHold());

        // AutoArmScoreLow
        codriver_B.onTrue(new AutoArmScoreLow())
            .onFalse(new ArmStowHold());

        // AutoArmRetrieveMedium
        codriver_RightBumper.onTrue(new AutoArmRetrieveMedium())
            .onFalse(new ArmStowHoldDelay());

        // AutoArmRetrieveLow
        codriver_RightTrigger.onTrue(new AutoArmRetrieveLow())
            .onFalse(new ArmStowHoldDelay());

        // GamePieceToggle
        codriver_Start.onTrue(new ToggleGamePiece());
    }

    private void bind_POVTest() {
      RobotShared robotShared = RobotShared.getInstance();

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
        .onTrue(new InstantCommand(() -> robotShared.toggleGamePiece()));
    }
    
    private void bind_CircleTest() {
      RobotShared robotShared = RobotShared.getInstance();
      DriveSubsystem m_robotDrive = robotShared.getDriveSubsystem();

      // Turn to -90 degrees with a profile when the Circle button is pressed, with a
      // 5 second timeout
      m_driverController.a()
          .onTrue(new TurnToAngleProfiled(90, m_robotDrive).withTimeout(5));
    }
    
    private void bind_AutoDriveDistanceTest() {
      RobotShared robotShared = RobotShared.getInstance();
      DriveSubsystem m_robotDrive = robotShared.getDriveSubsystem();

      // Auto-drive distance
      m_driverController.b()
        .onTrue(new DriveToDistance(5, m_robotDrive));
    }

    private void bind_ManualArmTest() {
      RobotShared robotShared = RobotShared.getInstance();
      ArmProfiledPIDSubsystem m_armProfiled = robotShared.getArmProfiledPIDSubsystem();
      TelescopePIDSubsystem m_telescope = robotShared.getTelescopePIDSubsystem();

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
      RobotShared robotShared = RobotShared.getInstance();
      DriveSubsystem m_robotDrive = robotShared.getDriveSubsystem();

      // Manually rsest the gyro
      m_driverController.start()
        .onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));
    }
  
    private void bind_GroundIntakeTest() {
      RobotShared robotShared = RobotShared.getInstance();
      GroundIntakeSubsystem m_groundIntake = robotShared.getGroundIntakeSubsystem();

      m_driverController.rightBumper()
        // This command must also stow the arm first!!
        .onTrue(new InstantCommand(() -> m_groundIntake.deploy(GroundIntakeConstants.kCubeIntakeSpeed)))
        .onFalse(new InstantCommand(() -> m_groundIntake.stow()));
  
      m_driverController.leftBumper()
        .onTrue(new InstantCommand(() -> m_groundIntake.eject(GroundIntakeConstants.kCubeEjectSpeed)))
        .onFalse(new InstantCommand(() -> m_groundIntake.stow()));
    }
  
    private void bind_Limelight() {
      RobotShared robotShared = RobotShared.getInstance();
      LimeLightSubsystem m_limelight = robotShared.getLimeLightSubsystem();

      m_driverController.y()
        .onTrue(new InstantCommand(() -> m_limelight.enableVisionProcessing()));
    }
  
    private void bind_HalfSpeed() {
      RobotShared robotShared = RobotShared.getInstance();
      DriveSubsystem m_robotDrive = robotShared.getDriveSubsystem();

      // Drive at half speed when the right bumper is held
      m_driverController.rightBumper()
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
    }
  
    // private void bind_PathWeaver() {
    //   RobotShared robotShared = RobotShared.getInstance();
    //   DriveSubsystem m_robotDrive = robotShared.getDriveSubsystem();

    // //   m_driverController.x()
    // //     .onTrue(m_robotDrive.getPathWeaverCommand(true));
    // // }
  
    /* ***************** CO-Driver Contols ************ */
    private void bind_CoAutoBalance() {
      RobotShared robotShared = RobotShared.getInstance();
      DriveSubsystem m_robotDrive = robotShared.getDriveSubsystem();

      // Drive to autobalance on teetertotter when 'X' button is pressed on codriver
      // controller, 5 second timeout
      m_codriverController.back()
        .onTrue(new AutoBalancePID(m_robotDrive));
    }
  
    private void bind_CoLightToggle() {
      RobotShared robotShared = RobotShared.getInstance();
      LimeLightSubsystem m_limelight = robotShared.getLimeLightSubsystem();

      // When codriver button is pressed, toggle the light
      m_codriverController.y()
        .onTrue(new InstantCommand(() -> m_limelight.toggleLED()));
    }
    
    private void bind_CoCubeOp(){
      RobotShared robotShared = RobotShared.getInstance();
      ClawSubsystem m_claw = robotShared.getClawSubsystem();

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
      RobotShared robotShared = RobotShared.getInstance();
      TelescopePIDSubsystem m_telescope = robotShared.getTelescopePIDSubsystem();
      ArmProfiledPIDSubsystem m_armProfiled = robotShared.getArmProfiledPIDSubsystem();
      ClawSubsystem m_claw = robotShared.getClawSubsystem();

      // Combination PID commands for Arm rotate & extend/retract
      m_codriverController.a()
        .onTrue(new SequentialCommandGroup(
          new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kStowedPosition)),
          // new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kStowedAngle))
          new InstantCommand(() -> m_armProfiled.setGoal(ArmConstants.kStowedAngle))));
  
      m_codriverController.b()
        .onTrue(new SequentialCommandGroup(
          // new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kHighNodeAngle)),
          new InstantCommand(() -> m_telescope.setSetpoint(robotShared.calculateTeleSetpoint(AutoMode.HIGH))),
          new InstantCommand(() -> m_armProfiled.setGoal(robotShared.calculateArmSetpoint(AutoMode.HIGH)))));
  
      m_codriverController.x()
        .onTrue(new SequentialCommandGroup(
          // new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kMidNodeAngle)),
          new InstantCommand(() -> m_armProfiled.setGoal(robotShared.calculateArmSetpoint(AutoMode.MID))),
          new InstantCommand(() -> m_telescope.setSetpoint(robotShared.calculateTeleSetpoint(AutoMode.MID)))));
  
      m_codriverController.y()
        .onTrue(new SequentialCommandGroup(
          // new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kLowNodeAngle)),
          new InstantCommand(() -> m_armProfiled.setGoal(robotShared.calculateArmSetpoint(AutoMode.LOW)))));
          // new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kLowNodePosition)));
  
    }
  
    private void bind_LedModeTest() {
      RobotShared robotShared = RobotShared.getInstance();
      SignalLEDSubsystem m_signalLEDs = robotShared.getSignalLEDSubsystem();
      ClawSubsystem m_claw = robotShared.getClawSubsystem();

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
      RobotShared robotShared = RobotShared.getInstance();
      SignalLEDSubsystem m_signalLEDs = robotShared.getSignalLEDSubsystem();

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
  
  
  
  
    
      
  
  
  
    
    
    
    
    
    
    



}
