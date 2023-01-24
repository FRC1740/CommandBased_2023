// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax m_rotationLeader = new CANSparkMax(ArmConstants.kRotationLeaderMotorPort, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rotationFollower = new CANSparkMax(ArmConstants.kRotationFollowerMotorPort, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_extensionMotor = new CANSparkMax(ArmConstants.kExtensionMotorPort, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder m_rotationEncoder;
  private final RelativeEncoder m_extensionEncoder;

  public Arm() {
    /** Creates a new Arm. */  
    m_rotationFollower.follow(m_rotationLeader);
    m_rotationEncoder = m_rotationLeader.getEncoder();
    m_extensionEncoder = m_extensionMotor.getEncoder();
  }

  public void Extend(double speed) {
    m_extensionMotor.set(speed);
  }

  public void Rotate(double speed) {
    m_rotationLeader.set(speed);
  }

  public double getAngle() {
    return m_rotationEncoder.getPosition();
  }

  public double getExtension() {
    return m_extensionEncoder.getPosition();
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
}
