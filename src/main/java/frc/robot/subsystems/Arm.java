// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.board.ArmTab;
import frc.constants.ArmConstants;
import frc.network.ArmTable;


public class Arm extends SubsystemBase {
  private final CANSparkMax m_rotationLeader = new CANSparkMax(ArmConstants.kRotationLeaderMotorPort, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rotationFollower = new CANSparkMax(ArmConstants.kRotationFollowerMotorPort, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_extensionMotor = new CANSparkMax(ArmConstants.kExtensionMotorPort, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder m_rotationEncoder;
  private final RelativeEncoder m_rotationFollowerEncoder;
  private final RelativeEncoder m_extensionEncoder;
  protected double m_setpoint;
  
  private ArmTab m_ArmTab;
  private ArmTable m_ArmTable;

  public Arm() {
    /** Creates a new Arm. */
    // The target angle for PID rotation control
    m_setpoint = ArmConstants.kStowedAngle;
    // Follower motor direction is inverted
    m_rotationFollower.follow(m_rotationLeader, true);
    m_rotationEncoder = m_rotationLeader.getEncoder();
    m_rotationFollowerEncoder = m_rotationFollower.getEncoder();
    m_extensionEncoder = m_extensionMotor.getEncoder();

    // Reset encoders to Zero position for starting configuration
    m_rotationEncoder.setPosition(0.0);
    m_extensionEncoder.setPosition(0.0);

    m_rotationEncoder.setPositionConversionFactor(ArmConstants.ARM_ROTATION_POSITION_CONVERSION_FACTOR);
    m_rotationFollowerEncoder.setPositionConversionFactor(ArmConstants.ARM_ROTATION_POSITION_CONVERSION_FACTOR);
    m_extensionEncoder.setPositionConversionFactor(ArmConstants.ARM_EXTENSION_POSITION_CONVERSION_FACTOR);
    m_rotationLeader.burnFlash();
    m_rotationFollower.burnFlash();
    
    m_ArmTab = ArmTab.getInstance();
    m_ArmTab.setArmAngle(getArmRotationDegrees());
    m_ArmTab.setArmExtension(getArmExtensionInches());

    m_ArmTable = ArmTable.getInstance();
       
  }

  public void setSetpoint(double angle) {
    m_setpoint = angle;
  }
  
  public void Extend(double speed) {
    m_extensionMotor.set(speed);
  }

  public void Rotate(double speed) {
    m_rotationLeader.set(speed);
  }

  public double getRotationEncoder() {
    return m_rotationEncoder.getPosition();
  }

  public double getRotationAngle() {
    return getRotationEncoder();
  }

  public double getExtensionEncoder() {
    return m_extensionEncoder.getPosition();
  }

  public double getExtensionDistanceInches() {
    return getExtensionEncoder();
  }

  public void resetRotationEncoder() {
    m_rotationEncoder.setPosition(0.0);
  }

  public void resetExtensionEncoder() {
    m_extensionEncoder.setPosition(0.0);
  }

  public void stopRotation() {
    m_rotationLeader.set(0.0);
  }

  public void stopExtension() {
    m_extensionMotor.set(0.0);
  }

  public void stopAll() {
    stopRotation();
    stopExtension();
  }

  private double getArmRotationEncoder() {
    return m_rotationEncoder.getPosition();
  }

  private double getArmExtensionEncoder() {
    return m_extensionEncoder.getPosition();

  }

  // Convert Encoder ticks to inches
  private double getArmExtensionInches() {
    return getArmExtensionEncoder();
  }
  // Convert Encoder ticks to degrees 
  private double getArmRotationDegrees() {
    return getArmRotationEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ArmTab.setArmAngle(getArmRotationDegrees());
    m_ArmTab.setArmExtension(getArmExtensionInches());          
  }

}
