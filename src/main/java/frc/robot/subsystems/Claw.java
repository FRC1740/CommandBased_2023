// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.constants.ClawConstants;
//import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.revrobotics.RelativeEncoder;
import frc.constants.ShuffleboardConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.*;

public class Claw extends SubsystemBase {
  private final DoubleSolenoid m_grabberSolenoid;
  private final WPI_TalonSRX m_intakeMotor; // One Talon controlling TWO bag motors (hardwired)
  // private final RelativeEncoder m_intakeEncoder;

  // public static final int kLedLength = 13;
  // public static final int kLedPwmPort = 3;
  // // Must be a PWM header, not MXP or DIO
  // private final AddressableLED m_led;
  // private final AddressableLEDBuffer m_ledBuffer;
  // private int m_delay;

  // public enum LedMode {
  //   CUBE,
  //   CONE,
  //   KITT,
  //   RED,
  //   OFF,
  // }

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
  // Shuffleboard DriveTrain entries
  // Create and get reference to SB tab
  private ShuffleboardTab m_sbt_Claw;

  // Parameters Passed from DS via Shuffleboard
  private GenericEntry m_nte_ClawMode;
  // private GenericEntry m_nte_IntakeSpeed;
  private GenericEntry m_nte_IntakeCurrent;

  /** Creates a new Manipulator. */
  public Claw() {
    m_grabberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClawConstants.kPneumaticPortA, ClawConstants.kPneumaticPortB);
    // m_intakeMotor = new CANSparkMax(ClawConstants.IntakeMotorCANID, CANSparkMax.MotorType.kBrushless);
    m_intakeMotor = new WPI_TalonSRX(ClawConstants.IntakeMotorCANID);
    m_intakeMotor.configPeakCurrentLimit(ClawConstants.IntakePeakCurrentLimit, 10); // Amps, timeout (msec)
    m_intakeMotor.configPeakCurrentDuration(ClawConstants.IntakePeakDurationLimit, 10); // msec, timeout
    m_intakeMotor.configContinuousCurrentLimit(ClawConstants.IntakeContinuousCurrentLimit, 10); // Amps, timeout
    // m_intakeEncoder = m_intakeMotor.getEncoder();
    // Set the colors appropriate for each game piece
    // cube = new ConSignalLed.gamePiece(50, 0, 100);
    // cone = new ConSignalLed.gamePiece(100, 50, 0);
    m_clawMode = ClawMode.READY;
    m_timer = new Timer();

    // Create and get reference to SB tab
    m_sbt_Claw = Shuffleboard.getTab(ShuffleboardConstants.ClawTab);

    // Create Widges for CURRENT Arm Position & Angle
    m_nte_ClawMode = m_sbt_Claw.addPersistent("Claw Mode", getModeString())
          .withSize(2, 1).withPosition(0, 0).getEntry();
    // m_nte_IntakeSpeed = m_sbt_Claw.addPersistent("Intake Speed", getIntakeSpeed())
    //       .withSize(2, 1).withPosition(0, 1).getEntry();
    m_nte_IntakeCurrent = m_sbt_Claw.addPersistent("Intake Current", getIntakeCurrent())
          .withSize(2, 1).withPosition(0, 2).getEntry();

  //   m_currentPixel = 0;
  //   m_kittDelta = 1;
  //   m_delay = 50; // Include a delay during Peridoc() if too processor intensive
  //   m_led = new AddressableLED(kLedPwmPort);
  //   m_ledBuffer = new AddressableLEDBuffer(kLedLength);
  //   // Length is expensive to set, so only set it once, then just update data
  //   m_led.setLength(m_ledBuffer.getLength());
    
  //   // Both LED strips MUST Be the same length
  //   m_led.start();
  //   m_led.setData(m_ledBuffer);
  //   m_timer.start();
  }

  // The actual robot may have TWO separate mechanisms for cone/cube
  // selected by the drive team via OI input TBD
  // NOTE: The Grab() and Release() methods may do different things depending on the 
  // gamepiece mode (cube/cone)
  public void intakeCube() {
    m_intakeMotor.set(ClawConstants.InjectCubeHighSpeed);
    setMode(ClawMode.CUBE);
  }
  public void ejectCube() {
    m_intakeMotor.set(ClawConstants.EjectCubeSpeed);
    m_timer.reset(); // Reset timer to allow a delayed shutdown of Eject Motors
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
    m_grabberSolenoid.set(kForward); 
    setMode(ClawMode.CONE);
    }
  private void Open() { // Open to release a cone or intake a cube
    m_grabberSolenoid.set(kReverse); 
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
    m_nte_ClawMode.setString(getModeString());
  }
  private ClawMode getMode() {
    return m_clawMode;
  }
  private String getModeString() {
    // return m_clawMode.toString();
    return m_clawMode.name();
  }
  // private void ShowLedPattern() {
  //   // Note Colors are ACTUALLY in RBG order!!!
  //   for (var i=0; i<m_ledBuffer.getLength(); i++) {
  //     switch(m_ledMode) {
  //       case CUBE: // Purplish (dark magenta)
  //         m_ledBuffer.setRGB(i, cube.getRed(), cube.getBlue(), cube.getGreen()); // Note: RBG for our LEDs
  //         break;
  //       case CONE: // Yellow-orange
  //         m_ledBuffer.setRGB(i, cone.getRed(), cone.getBlue(), cone.getGreen());  // Note: RBG for our LEDs
  //         break;
  //       case RED: // Error mode
  //         m_ledBuffer.setRGB(i, 255, 0, 0);  // Note: RBG for our LEDs
  //         break;
  //       case KITT: 
  //         Kitt(); // Cylon Pattern
  //         break;
  //       case OFF:
  //         m_ledBuffer.setRGB(i, 0, 0, 0);  // Note: RBG for our LEDs
  //         break;
  //     } 
  //     m_led.setData(m_ledBuffer);
  //   }
  // }

  // /* Disabled (orange) is too close to cone color  */
  // public void Kitt() { 
  //   for (int i = 0; i < m_ledBuffer.getLength(); i++) {
  //     Color c = m_ledBuffer.getLED(i);
  //     int r = (int)Math.round(c.red * 255);
  //     int g = (int)Math.round(c.green * 255);
  //     int b = (int)Math.round(c.blue * 255);

  //     if (r >= 10) r -= 10; else r = 0;
  //     if (g >= 20) g -= 20; else g = 0;
  //     if (b >= 20) b -= 20; else b = 0;
  //     m_ledBuffer.setRGB(i, r, b, g);
  //   }
  //   m_ledBuffer.setRGB(m_currentPixel, 64, 64, 64);
  //   m_led.setData(m_ledBuffer);
  //   // m_ledB.setData(m_ledBuffer);

  //   m_currentPixel += m_kittDelta;
  //   if ((m_currentPixel <= 0) || (m_currentPixel >= kLedLength - 1)) {
  //     // Ensure valid even when switching modes
  //     if (m_currentPixel < 0) m_currentPixel = 0;
  //     if (m_currentPixel > kLedLength - 1) m_currentPixel = kLedLength - 1;
  //     m_kittDelta = -m_kittDelta;
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // ShowLedPattern();
    // if (--m_delay == 0) {
    //   m_delay = 50;
    // }
    // Shutdown the Cube Eject Motor after a delay if we're not intaking a cube
    if (m_timer.get() > ClawConstants.ShutdownDelay) {
      if (m_clawMode == ClawMode.CUBE) {
        m_intakeMotor.set(ClawConstants.InjectCubeLowSpeed);
      } else {
        m_intakeMotor.set(0.0);
      }
    }
    // m_nte_IntakeSpeed.setDouble(getIntakeSpeed());
    m_nte_IntakeCurrent.setDouble(getIntakeCurrent());
  }
}
