// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.board.ArmTab;
import frc.board.AutonomousTab;
import frc.board.RobotTab;
import frc.constants.ArmConstants;
import frc.constants.OIConstants;
import frc.constants.OIConstants.GamePiece;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SignalLEDSubsystem;
import frc.robot.subsystems.SignalLEDSubsystem.LedMode;
import frc.robot.subsystems.SignalLEDSubsystem.LedPreference;
import frc.robot.subsystems.TelescopePIDSubsystem;

/** Add your docs here. */
public class RobotShared {

    private static RobotShared instance;

    // The robot's subsystems and commands are defined here...
    protected DriveSubsystem m_robotDrive = null;
    protected LimeLightSubsystem m_limelight = null;
    protected PhotonVisionSubsystem m_photonVision = null;
    protected ClawSubsystem m_claw = null;
    // protected final ArmPIDSubsystem m_arm = new ArmPIDSubsystem();
    protected ArmProfiledPIDSubsystem m_armProfiled = null;
    protected TelescopePIDSubsystem m_telescope = null;
    protected GroundIntakeSubsystem m_groundIntake = null;
    protected SignalLEDSubsystem m_signalLEDs = null;

    //I dont know if this is how you're supposed to do this
    protected Paths m_paths = null;


    protected final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    protected final CommandXboxController m_codriverController = new CommandXboxController(OIConstants.kCoDriverControllerPort);

    private RobotTab m_robotTab;
    private ArmTab m_armTab;

    private RobotShared() {
        m_robotTab = RobotTab.getInstance();
        m_armTab = ArmTab.getInstance();
    }

    public static RobotShared getInstance() {
        if(instance == null) {
            instance = new RobotShared();
        }
        return instance;
    }

    public DriveSubsystem getDriveSubsystem() {
        if(m_robotDrive == null) {
            m_robotDrive = new DriveSubsystem();
        }
        return m_robotDrive;
    }

    public LimeLightSubsystem getLimeLightSubsystem() {
        if(m_limelight == null) {
            m_limelight = new LimeLightSubsystem();
        }
        return m_limelight;
    }

    public PhotonVisionSubsystem getPhotonVisionSubsystem() {
        if(m_photonVision == null) {
            m_photonVision = new PhotonVisionSubsystem();
        }
        return m_photonVision;
    }

    public ClawSubsystem getClawSubsystem() {
        if(m_claw == null) {
            m_claw = new ClawSubsystem();
        }
        return m_claw;
    }

    public ArmProfiledPIDSubsystem getArmProfiledPIDSubsystem() {
        if(m_armProfiled == null) {
            m_armProfiled = new ArmProfiledPIDSubsystem();
        }
        return m_armProfiled;
    }

    public TelescopePIDSubsystem getTelescopePIDSubsystem() {
        if(m_telescope == null) {
            m_telescope = new TelescopePIDSubsystem();
        }
        return m_telescope;
    }

    public GroundIntakeSubsystem getGroundIntakeSubsystem() {
        if(m_groundIntake == null) {
            m_groundIntake = new GroundIntakeSubsystem();
        }
        return m_groundIntake;
    }

    public SignalLEDSubsystem getSignalLEDSubsystem() {
        if(m_signalLEDs == null) {
            m_signalLEDs = new SignalLEDSubsystem();
        }
        return m_signalLEDs;
    }

    public Paths getPaths(){
        if(m_paths == null) {
            m_paths = new Paths();
        }
        return m_paths;
    }

    public CommandXboxController getDriverController() {
        return m_driverController;
    }

    public CommandXboxController getCodriverController() {
        return m_codriverController;
    }

    public RobotTab getRobotTab() {
        return m_robotTab;
    }

    // This seems to be the best place- knows about GamePiece and Arm
    public double calculateTeleSetpoint(ArmConstants.AutoMode mode) {
        GamePiece piece = m_robotTab.getGamePiece();
        switch (mode) {
            case HIGH:
                if (piece == OIConstants.GamePiece.CONE)
                    return m_armTab.getConeHighPosition();
                else
                    return m_armTab.getCubeHighPosition();
            case MID:
                if (piece == OIConstants.GamePiece.CONE)
                    return m_armTab.getConeMidPosition();
                else
                    return m_armTab.getCubeMidPosition();
            case LOW:
                if (piece == OIConstants.GamePiece.CONE)
                    return m_armTab.getConeLowPosition();
                else
                    return m_armTab.getCubeLowPosition();
            case SHELF:
                if (piece == OIConstants.GamePiece.CONE)
                    return m_armTab.getConeShelfPosition();
                else
                    return m_armTab.getCubeShelfPosition();
            case FLOOR:
                if (piece == OIConstants.GamePiece.CONE)
                    return m_armTab.getConeFloorPosition();
                else
                    return m_armTab.getCubeFloorPosition();
            case STOWED:
            default:
                return ArmConstants.kStowedPosition;
        }
    }

    public double calculateArmSetpoint(ArmConstants.AutoMode mode) {
        GamePiece piece = m_robotTab.getGamePiece();
        switch (mode) {
            case HIGH:
                if (piece == OIConstants.GamePiece.CONE)
                    return m_armTab.getConeHighAngle();
                else
                    return m_armTab.getCubeHighAngle();
            case MID:
                if (piece == OIConstants.GamePiece.CONE)
                    return m_armTab.getConeMidAngle();
                else
                    return m_armTab.getCubeMidAngle();
            case LOW:
                if (piece == OIConstants.GamePiece.CONE)
                    return m_armTab.getConeLowAngle();
                else
                    return m_armTab.getCubeLowAngle();
            case SHELF:
                if (piece == OIConstants.GamePiece.CONE)
                    return m_armTab.getConeShelfAngle();
                else
                    return m_armTab.getCubeShelfAngle();
            case FLOOR:
                if (piece == OIConstants.GamePiece.CONE)
                    return m_armTab.getConeFloorAngle();
                else
                    return m_armTab.getCubeFloorAngle();
            case STOWED:
            default:
                return ArmConstants.kStowedAngle;
        }
    }

    public double calculateRelativeArmSetpoint() {
        GamePiece piece = m_robotTab.getGamePiece();
        if (piece == OIConstants.GamePiece.CONE)
            return m_armTab.getConeDunkAngle();
        else
            return m_armTab.getCubeDunkAngle();
    }

    public double calculateAutoArmScoreDelay() {
        GamePiece piece = m_robotTab.getGamePiece();
        if (piece == OIConstants.GamePiece.CONE)
            return ArmConstants.kAutoArmScoreConeDelay;
        else
            return ArmConstants.kAutoArmScoreCubeDelay;
    }

    public double calculateArmRotateRelativeDelay() {
        GamePiece piece = m_robotTab.getGamePiece();
        if (piece == OIConstants.GamePiece.CONE)
            return ArmConstants.kArmRotateRelativeConeDelay;
        else
            return ArmConstants.kArmRotateRelativeCubeDelay;
    }

    public double calculateDunkScoreDelay() {
        GamePiece piece = m_robotTab.getGamePiece();
        if (piece == OIConstants.GamePiece.CONE)
            return ArmConstants.kDunkScoreConeDelay;
        else
            return ArmConstants.kDunkScoreCubeDelay;
    }

    public void toggleGamePiece() {
        GamePiece newGamePiece = m_robotTab.toggleGamePiece();
        setGamePiece(newGamePiece);
      }
    
    public void setGamePiece(OIConstants.GamePiece piece) {
        SignalLEDSubsystem signalLEDs = getSignalLEDSubsystem();
        GroundIntakeSubsystem groundIntake = getGroundIntakeSubsystem();
        ClawSubsystem claw = getClawSubsystem();

        if (piece == OIConstants.GamePiece.CUBE) {
            signalLEDs.setMode(LedMode.CUBE, LedPreference.MAIN, false);
        } else if (piece == OIConstants.GamePiece.CONE) {
            signalLEDs.setMode(LedMode.CONE, LedPreference.MAIN, false);
        }
        groundIntake.setGamePiece(piece);
        claw.setGamePiece(piece);
        AutonomousTab.getInstance().setGamePiece(piece);
    }
    
    public GamePiece getGamePiece() {
        return m_robotTab.getGamePiece();
    }
}
