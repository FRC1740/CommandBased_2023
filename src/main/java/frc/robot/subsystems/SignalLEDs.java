// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.*;

public class SignalLEDs extends SubsystemBase {

  public enum LedMode {
    CUBE,
    CONE,
    KITT,
    RED,
    OFF,
  };

  // Caller's preference when changing mode
  public enum LedPreference {
    MAIN,  // Higher priority
    ALT,   // If INDEPENDENT, use second string else NOP
  }

  // We can support multiple LED strings but fall back
  // to a single one without client awareness
  public enum LedHwConfig {
    SINGLE,      // MAIN preference uses first string
    //PARALLEL,    // Two strings, MAIN acts like single on both
    //SERIES,      // Two strings, MAIN acts like single end-to-end
    INDEPENDENT, // Independently controlled strings
    OFF,         // Disable LED support
  };

  public LedHwConfig m_hwConfig = LedHwConfig.SINGLE;

  public static final int kLedLengthA = 13;
  public static final int kLedPwmPortA = 3;

  public static final int kLedLengthB = 13;
  public static final int kLedPwmPortB = 4;

  // Common to all strings
  public static final int kRefreshEvery = 30;
  ConSignalLed.gamePiece cube, cone;  // Stores RGB triplets

  // Encapsulate the details for each physical LED string here
  private class LedHwString {
    int m_port;
    int m_length;
    // Must be a PWM header, not MXP or DIO
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer; 
    int m_direction;        // Direction indicator for non-solid
    int m_currentPixel;     // Used if mode is non-solid
    LedMode m_ledMode;      // Selected mode
    LedMode m_defaultMode;  // Default (persisted) mode

    private LedHwString(int port, int length) {
      m_port = port;
      m_length = length;
      m_led = new AddressableLED(port);
      m_ledBuffer = new AddressableLEDBuffer(length);

      // Length is expensive to set, so only set it once, then just update data
      m_led.setLength(length);
      
      m_ledMode = LedMode.OFF;
      m_defaultMode = LedMode.OFF;
      m_led.start();
      m_led.setData(m_ledBuffer);
      System.out.println("Members"+m_port+" "+m_length+" "+m_delay+" "+m_ledMode+" "+m_direction+" "+m_currentPixel);
    }
  }

  private int m_delay;  // Time between update refreshes
  private LedHwString[] m_hwStrings;

  /** Create new SignalLED(s) */
  public SignalLEDs() {
    // Define the colors appropriate for each game piece
    cube = new ConSignalLed.gamePiece(50, 0, 100);
    cone = new ConSignalLed.gamePiece(100, 50, 0);

    m_delay = kRefreshEvery;

    // Create the strings, based on hardware
    if (m_hwConfig == LedHwConfig.OFF) return;
    if (m_hwConfig == LedHwConfig.SINGLE) {
      System.out.println("Configure one string");
      m_hwStrings = new LedHwString[1];
      m_hwStrings[0] = new LedHwString(kLedPwmPortA, kLedLengthA);
    }
    else {
      System.out.println("Configure two strings");
      m_hwStrings = new LedHwString[2];
      m_hwStrings[0] = new LedHwString(kLedPwmPortA, kLedLengthA);
      m_hwStrings[1] = new LedHwString(kLedPwmPortB, kLedLengthB);
    }
  }

  private void ShowLedPattern(LedHwString hwString) {
    // Note Colors are ACTUALLY in RBG order!!!
    for (var i=0; i<hwString.m_ledBuffer.getLength(); i++) {
      switch(hwString.m_ledMode) {
        case CUBE: // Purplish (dark magenta)
        hwString.m_ledBuffer.setRGB(i, cube.getRed(), cube.getBlue(), cube.getGreen()); // Note: RBG for our LEDs
          break;
        case CONE: // Yellow-orange
        hwString.m_ledBuffer.setRGB(i, cone.getRed(), cone.getBlue(), cone.getGreen());  // Note: RBG for our LEDs
          break;
        case RED: // Error mode
        hwString.m_ledBuffer.setRGB(i, 255, 0, 0);  // Note: RBG for our LEDs
          return;
        case KITT: 
          Kitt(); // Cylon Pattern
          break;
        case OFF:
        hwString.m_ledBuffer.setRGB(i, 0, 0, 0);  // Note: RBG for our LEDs
          break;
      } 
      hwString.m_led.setData(hwString.m_ledBuffer);
    }
  }

  public void Kitt() { 
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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (--m_delay <= 0) {
      m_delay = kRefreshEvery;
      ShowLedPattern(m_hwStrings[0]);
    }
  }

  /*  Implement this Logic
      if a string does not exist at the index
        return
      else
        if background is true
          if currentMode is same as defaultdMode
            replace currentMode with newMode
          replace defaultMode with newMode
          return
        else
          if newMode is same as currentMode
            return
          else
            if newMode is OFF
              replace currentMode with defaultMode
            else
              replace currentMode with newMode
   */
  public void setMode(LedMode newMode, LedPreference preference, boolean background) {
    System.out.println("Change LED mode to " + newMode);
    LedHwString hwString;
  
    switch (m_hwConfig) {
      case SINGLE:
        if (preference != LedPreference.MAIN)
          break;
        hwString = m_hwStrings[0];
        if (background) {
          if (hwString.m_ledMode == hwString.m_defaultMode)
            hwString.m_ledMode = newMode;
          hwString.m_defaultMode = newMode;
          break;
        }
        if (hwString.m_ledMode == newMode)
          break;
        if (newMode == LedMode.OFF) {
          hwString.m_ledMode = hwString.m_defaultMode;
          break;
        }
        hwString.m_ledMode = newMode;
      break;

      case INDEPENDENT:
      // No support yet
      break;

      case OFF:
      // No HW strings
      break;
    }
    // m_ledMode = newMode;
  }
}
