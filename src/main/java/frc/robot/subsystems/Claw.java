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
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;
import frc.constants.ClawConstants;
import com.revrobotics.CANSparkMax;

public class Claw extends SubsystemBase {
  DoubleSolenoid m_grabberSolenoid;
  CANSparkMax m_intakeMotor;

  public static final int kLedLength = 13;
  public static final int kLedPwmPort = 3;
  // Must be a PWM header, not MXP or DIO
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private int m_delay;

  public enum LedMode {
    CUBE,
    CONE,
    KITT,
    RED,
    OFF,
  };    

  private LedMode m_mode;
  ConSignalLed.gamePiece cube, cone;
  private int m_currentPixel;
  private int m_kittDelta;
  private Timer m_timer;

  /** Creates a new Manipulator. */
  public Claw() {
    m_grabberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClawConstants.kPneumaticPortA, ClawConstants.kPneumaticPortB);
    m_intakeMotor = new CANSparkMax(ClawConstants.IntakeMotorCANID, CANSparkMax.MotorType.kBrushless);
    // Set the colors appropriate for each game piece
    cube = new ConSignalLed.gamePiece(50, 0, 100);
    cone = new ConSignalLed.gamePiece(100, 50, 0);

    m_currentPixel = 0;
    m_kittDelta = 1;
    m_delay = 50; // Include a delay during Peridoc() if too processor intensive
    m_led = new AddressableLED(kLedPwmPort);
    m_ledBuffer = new AddressableLEDBuffer(kLedLength);
    // Length is expensive to set, so only set it once, then just update data
    m_led.setLength(m_ledBuffer.getLength());
    
    // Both LED strips MUST Be the same length
    m_mode = LedMode.CUBE;
    m_led.start();
    m_led.setData(m_ledBuffer);
  }

  // The actual robot may have TWO separate mechanisms for cone/cube
  // selected by the drive team via OI input TBD
  // NOTE: The Grab() and Release() methods may do different things depending on the 
  // gamepiece mode (cube/cone)

  public void Toggle() {
    switch(m_mode) {
      case CUBE: // We're currently set for a cube
        m_grabberSolenoid.set(kReverse);
        m_mode = LedMode.CONE;
        break;
      case CONE: // We're currently set for a cone
        m_grabberSolenoid.set(kForward);
        m_mode = LedMode.CUBE;
        default:
        break;
    }
  }

  public void Close() { // Close to grab a cone
    m_grabberSolenoid.set(kForward); 
  }
  public void Open() { // Open to release a cone or intake a cube
    m_grabberSolenoid.set(kReverse); 
  }

  public void setMode(LedMode newMode) {
    m_mode = newMode;
  }

  private void ShowLedPattern() {
    // Note Colors are ACTUALLY in RBG order!!!
    for (var i=0; i<m_ledBuffer.getLength(); i++) {
      switch(m_mode) {
        case CUBE: // Purplish (dark magenta)
          m_ledBuffer.setRGB(i, cube.getRed(), cube.getBlue(), cube.getGreen()); // Note: RBG for our LEDs
          break;
        case CONE: // Yellow-orange
          m_ledBuffer.setRGB(i, cone.getRed(), cone.getBlue(), cone.getGreen());  // Note: RBG for our LEDs
          break;
        case RED: // Error mode
          m_ledBuffer.setRGB(i, 255, 0, 0);  // Note: RBG for our LEDs
          break;
        case KITT: 
          Kitt(); // Cylon Pattern
          break;
        case OFF:
          m_ledBuffer.setRGB(i, 0, 0, 0);  // Note: RBG for our LEDs
          break;
      } 
      m_led.setData(m_ledBuffer);
    }
  }

  /* Disabled (orange) is too close to cone color  */
  public void Kitt() { 
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      Color c = m_ledBuffer.getLED(i);
      int r = (int)Math.round(c.red * 255);
      int g = (int)Math.round(c.green * 255);
      int b = (int)Math.round(c.blue * 255);

      if (r >= 10) r -= 10; else r = 0;
      if (g >= 20) g -= 20; else g = 0;
      if (b >= 20) b -= 20; else b = 0;
      m_ledBuffer.setRGB(i, r, b, g);
    }
    m_ledBuffer.setRGB(m_currentPixel, 64, 64, 64);
    m_led.setData(m_ledBuffer);
    // m_ledB.setData(m_ledBuffer);

    m_currentPixel += m_kittDelta;
    if ((m_currentPixel <= 0) || (m_currentPixel >= kLedLength - 1)) {
      // Ensure valid even when switching modes
      if (m_currentPixel < 0) m_currentPixel = 0;
      if (m_currentPixel > kLedLength - 1) m_currentPixel = kLedLength - 1;
      m_kittDelta = -m_kittDelta;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ShowLedPattern();
    if (--m_delay == 0) {
      m_delay = 50;
      System.out.println(m_mode);
    }
    // Shutdown the Cube Eject Motor after a delay if we're not intaking a cube
    if (m_mode != LedMode.CUBE && m_timer.get() > ClawConstants.ShutdownDelay) {
      m_intakeMotor.set(0.0);
    }
  }
}
