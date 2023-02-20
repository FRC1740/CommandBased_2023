// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
// import frc.robot.Constants.*;
import frc.constants.LEDConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SignalLEDSubsystem extends SubsystemBase {

    public static class gamePiece {
      int red, green, blue;

      public gamePiece(int r, int g, int b) {
          red = r;
          green = g;
          blue = b;
      }
      public int getRed() {
          return red;
      }
      public int getGreen() {
          return green;
      }
      public int getBlue() {
          return blue;
      }
  }

  public enum LedMode {
    CUBE,
    CONE,
    RED,
    GREEN,
    ALLIANCE, // Show alliance color (Red/Blue/Green if unavailable)
    BLUE,
    KITT,
    COLONELS,
    COUNTDOWN,
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
  public static final int kRefreshEvery = 2;
  gamePiece cube, cone;  // Stores RGB triplets

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
      
      m_direction = 1;
      m_currentPixel = 0;
      m_ledMode = LedMode.OFF;
      m_defaultMode = LedMode.OFF;
      m_led.start();
      m_led.setData(m_ledBuffer);
      // System.out.println("Members "+m_port+" "+m_length+" "+m_delay+" "+m_ledMode+" "+m_direction+" "+m_currentPixel);
    }
  }

  private int m_delay;  // Time between update refreshes
  private int m_secondsTick;
  private LedHwString[] m_hwStrings;
  private Alliance m_alliance;
  private int m_matchTime;

  /** Create new SignalLED(s) */
  public SignalLEDSubsystem() {
    // Define the colors appropriate for each game piece
    cube = new gamePiece(LEDConstants.kCubeR, LEDConstants.kCubeG, LEDConstants.kCubeB);
    cone = new gamePiece(LEDConstants.kConeR, LEDConstants.kConeG, LEDConstants.kConeB);

    m_delay = kRefreshEvery;
    m_secondsTick = 0;

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
    int r=0, g=0, b=0; // Default to "OFF"
    switch (hwString.m_ledMode) {
      case CUBE: // Purplish (dark magenta)
        r=cube.getRed();
        b=cube.getBlue();
        g=cube.getGreen(); // Note: RBG for our LEDs
        break;
      case CONE: // Yellow-orange
        r=cone.getRed();
        b=cone.getBlue();
        g=cone.getGreen(); 
        break;
      case RED:
        r=255; b=0; g=0;  // Note: RBG for our LEDs
        break;
      case GREEN:
        r=0; b=0; g=255;  // Note: RBG for our LEDs
        break;
        case BLUE:
        r=0; b=255; g=0;  // Note: RBG for our LEDs
        break;
      case ALLIANCE: 
        if (m_alliance == Alliance.Red) {
          r=255; g=0; b=0;
        } 
        else if (m_alliance == Alliance.Blue) {
          r=0; g=0; b=255;
        }
        else { // Default to green if Alliance N/A
          r=0; g=255; b=0;
        }
        break;
      case KITT: 
        Kitt(hwString); // Cylon Pattern
        return;
      case COLONELS: 
        Colonels(hwString);
        return;
      case COUNTDOWN: 
        Countdown(hwString);
        return;
      case OFF:
      default:
        break;
    } 
    for (var i=0; i<hwString.m_length; i++) {
      hwString.m_ledBuffer.setRGB(i, r, b, g);  // Note: RBG for our LEDs
    }
    hwString.m_led.setData(hwString.m_ledBuffer);
  }

  public void Kitt(LedHwString hwString) { 
    for (int i = 0; i < hwString.m_length; i++) {
      Color c = hwString.m_ledBuffer.getLED(i);
      int r = (int)Math.round(c.red * 255);
      int g = (int)Math.round(c.green * 255);
      int b = (int)Math.round(c.blue * 255);

      if (r >= 10) r -= 10; else r = 0;
      if (g >= 20) g -= 20; else g = 0;
      if (b >= 20) b -= 20; else b = 0;
      hwString.m_ledBuffer.setRGB(i, r, b, g);
    }
    hwString.m_ledBuffer.setRGB(hwString.m_currentPixel, 64, 64, 64);
    hwString.m_led.setData(hwString.m_ledBuffer);

    hwString.m_currentPixel += hwString.m_direction;
    if ((hwString.m_currentPixel <= 0) || (hwString.m_currentPixel >= hwString.m_length - 1)) {
      // Ensure valid even when switching modes
      if (hwString.m_currentPixel < 0) hwString.m_currentPixel = 0;
      if (hwString.m_currentPixel > hwString.m_length - 1) hwString.m_currentPixel = hwString.m_length - 1;
      hwString.m_direction = -hwString.m_direction;
    }
  }

public void Colonels(LedHwString hwString) { 
    for (int i = 0; i < hwString.m_length; i++) {
      if (((i + hwString.m_currentPixel) % hwString.m_length) > hwString.m_length / 2) {
        hwString.m_ledBuffer.setRGB(i, 0, 255, 0);
      } else {
        hwString.m_ledBuffer.setRGB(i, 100, 100, 100);
      }
    }
    hwString.m_led.setData(hwString.m_ledBuffer);

    hwString.m_currentPixel += 1;
    if (hwString.m_currentPixel >= hwString.m_length - 1) {
      hwString.m_currentPixel = 0;
    }
  }

public void Countdown(LedHwString hwString) {
    if ((m_matchTime < 0) || (m_matchTime > 15)) return;
    int last = m_matchTime * hwString.m_length / 15;
    if (last < 0) last = 0;
    if (last > hwString.m_length - 1) last = hwString.m_length - 1;
    // System.out.println("last " + last);
    for (int i = 0; i < hwString.m_length; i++) {
      if (i >= last) {
        hwString.m_ledBuffer.setRGB(i, 0, 0, 0);
      } else {
        hwString.m_ledBuffer.setRGB(i, 100, 100, 100);
      }
    }
    hwString.m_led.setData(hwString.m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (--m_delay <= 0) {
      m_delay = kRefreshEvery;
      ShowLedPattern(m_hwStrings[0]);
    }
    if (--m_secondsTick <= 0) {
      m_secondsTick = 50;  // Assuming standard 20ms rate
      // Recommended to repeat getting alliance value to avoid missing a change
      m_alliance = DriverStation.getAlliance();
      m_matchTime = (int)(DriverStation.getMatchTime() + 0.5);

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
      // No HW strings- NOOP
      break;
    }
  }
}
