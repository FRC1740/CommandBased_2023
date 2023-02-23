// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.constants.OIConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

// Comment to force commit
// Using "import static an.enum.or.constants.inner.class.*;" helps reduce verbosity
// this replaces "DoubleSolenoid.Value.kForward" with just kForward
// further reading is available at https://www.geeksforgeeks.org/static-import-java/
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;
import frc.board.ClawTab;
import frc.constants.ClawConstants;
//import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.revrobotics.RelativeEncoder;

public class ClawSubsystem extends SubsystemBase {
  private final DoubleSolenoid m_grabberSolenoid;
  private final WPI_TalonSRX m_intakeMotor; // One Talon controlling TWO bag motors (hardwired)
  // private final RelativeEncoder m_intakeEncoder;

  public enum ClawMode {
    CUBE,
    CONE,
    READY,
  };    

  private ClawMode m_clawMode;
  // private LedMode m_ledMode;
  // ConSignalLed.gamePiece cube, cone;
  // private int m_currentPixel;
  // private int m_kittDelta;
  private Timer m_timer;
  private ClawTab m_ClawTab;

  private OIConstants.GamePiece m_gamePiece = OIConstants.kDefaultGamePiece;

  /** Creates a new Manipulator. */
  public ClawSubsystem() {
    m_grabberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClawConstants.kPneumaticPortA, ClawConstants.kPneumaticPortB);
    // m_intakeMotor = new CANSparkMax(ClawConstants.IntakeMotorCANID, CANSparkMax.MotorType.kBrushless);
    m_intakeMotor = new WPI_TalonSRX(ClawConstants.IntakeMotorCANID);
    m_intakeMotor.configPeakCurrentLimit(ClawConstants.IntakePeakCurrentLimit, 10); // Amps, timeout (msec)
    m_intakeMotor.configPeakCurrentDuration(ClawConstants.IntakePeakDurationLimit, 10); // msec, timeout
    m_intakeMotor.configContinuousCurrentLimit(ClawConstants.IntakeContinuousCurrentLimit, 10); // Amps, timeout
    // m_intakeEncoder = m_intakeMotor.getEncoder();

    m_clawMode = ClawMode.READY;
    m_timer = new Timer();

    m_ClawTab = ClawTab.getInstance();
    m_ClawTab.setClawMode(getModeString());
    m_ClawTab.setIntakeCurrent(getIntakeCurrent());

    m_gamePiece = OIConstants.kDefaultGamePiece;
  }

  // Gamepiece Cone/Cube is a global mode
  // FIXME: Claw has state based on Gamepiece and task. First create the state machine for each GamePiece; only then implement

  // NOTE: The Grab() and Release() methods may do different things depending on the 
  // gamepiece mode (cube/cone)
  public void intakeCube() {
    m_intakeMotor.set(ClawConstants.InjectCubeHighSpeed);
    Open();
    setMode(ClawMode.CUBE);
  }

  public void ejectCube() {
    m_intakeMotor.set(ClawConstants.EjectCubeSpeed);
   m_intakeMotor.setExpiration(1);
    setMode(ClawMode.READY);
  }

  public void grabCone() {
    m_intakeMotor.set(ClawConstants.InjectConeSpeed);
    close();
    setMode(ClawMode.CONE);
  }

  public void dropCone() {
    m_intakeMotor.set(0.0);
    Open();
  }

  public void grabOrReleaseCone() {
    if (getMode() == ClawMode.READY) {
      grabCone();
    }
    else {
      dropCone();
    }
  }

  public void grabOrReleaseCube() {
    if (getMode() == ClawMode.READY) {
      intakeCube();
    }
    else {
      ejectCube();
    }
  }

  public void toggle() {
    switch(m_clawMode) {
      case CUBE: // We're currently set for a cube (or "READY")
      case READY:
        close();
        break;
      case CONE: // We're currently set for a cone
        Open();
        default: // Should be no other modes, but do nothing in any case
        break;
    }
  }

  private void close() { // Close to grab a cone
    m_grabberSolenoid.set(kReverse); 
    setMode(ClawMode.CONE);
  }

  private void Open() { // Open to release a cone or intake a cube
    m_grabberSolenoid.set(kForward); 
    setMode(ClawMode.CUBE);
  }

  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(speed);
  }

  public double getIntakeCurrent() {
    return m_intakeMotor.getStatorCurrent();
  }

  private void setMode(ClawMode newMode) {
    m_clawMode = newMode;
    System.out.println(m_clawMode);
    m_ClawTab.setClawMode(getModeString());
  }

  private ClawMode getMode() {
    return m_clawMode;
  }

  private String getModeString() {
    // return m_clawMode.toString();
    return m_clawMode.name();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Shutdown the Cube Eject Motor after a delay if we're not intaking a cube
    if (m_timer.get() > ClawConstants.ShutdownDelay) {
      if (m_clawMode == ClawMode.CUBE) {
        m_intakeMotor.set(ClawConstants.InjectCubeLowSpeed);
      } else {
        m_intakeMotor.set(0.0);
      }
    }
    // m_nte_IntakeSpeed.setDouble(getIntakeSpeed());
    m_ClawTab.setIntakeCurrent(getIntakeCurrent());
  }

  public void setGamePiece(OIConstants.GamePiece gamePiece) {
    m_gamePiece = gamePiece;
  }

  public void burnFlash() {
    // only for SparkMax
  }

}
