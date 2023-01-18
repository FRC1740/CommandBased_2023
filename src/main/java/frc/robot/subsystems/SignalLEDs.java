// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.*;

public class SignalLEDs extends SubsystemBase {

  public static final int kLedLength = 13;
  public static final int kLedPwmPort = 3;
  // Must be a PWM header, not MXP or DIO
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private int m_delay;

  public enum mode {
    CUBE,
    CONE,
    OFF,
  };

  private mode m_mode;
  ConSignalLed.gamePiece cube, cone;

  /** Creates a new SignalLEDs. */
  public SignalLEDs() {
    // Set the colors appropriate for each game piece
    cube = new ConSignalLed.gamePiece(50, 0, 100);
    cone = new ConSignalLed.gamePiece(100, 50, 0);

    m_delay = 30;
    m_led = new AddressableLED(3);
    m_ledBuffer = new AddressableLEDBuffer(kLedLength);
    // Length is expensive to set, so only set it once, then just update data
    // m_led.setLength(m_ledBuffer.getLength());
    m_led.setLength(kLedLength);
    
    // Both LED strips MUST Be the same length
    m_mode = mode.CONE;
    m_led.start();
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (--m_delay == 0) {
      SolidColor();
      m_delay = 30;
    }
  }

  private void SolidColor() {
    // Note Colors are ACTUALLY in RBG order!!!
    for (var i=0; i<kLedLength; i++) {
      switch(m_mode) {
        case CUBE: // Purplish (dark magenta)
          m_ledBuffer.setRGB(i, cube.getRed(), cube.getBlue(), cube.getGreen());
          break;
        case CONE: // Yellow-orange
          m_ledBuffer.setRGB(i, cone.getRed(), cone.getBlue(), cone.getGreen());
          break;
        case OFF:
          m_ledBuffer.setRGB(i, 0, 0, 0);
          break;
      } 
      m_led.setData(m_ledBuffer);
    }
  }
  /* Disabled (orange) is too close to cone color  */
  /*
  public void Disabled() { 
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 255, 48, 0);
      }
      m_led.setData(m_ledBuffer);
    }
    
  */
  public void setMode(mode newMode) {
    m_mode = newMode;
  }
}
