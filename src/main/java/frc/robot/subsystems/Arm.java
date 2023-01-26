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
import frc.constants.ShuffleboardConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.*;


public class Arm extends SubsystemBase {
  private final CANSparkMax m_rotationLeader = new CANSparkMax(ArmConstants.kRotationLeaderMotorPort, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rotationFollower = new CANSparkMax(ArmConstants.kRotationFollowerMotorPort, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_extensionMotor = new CANSparkMax(ArmConstants.kExtensionMotorPort, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder m_rotationEncoder;
  private final RelativeEncoder m_extensionEncoder;

  // Used to grab an instance of the global network tables
  // NetworkTableInstance inst;
  // NetworkTable m_nt;
  // Shuffleboard m_sb;

  // Shuffleboard DriveTrain entries
  // Create and get reference to SB tab
  ShuffleboardTab m_sbt_Arm;

  // Encoders/PID Feedback sensors
  GenericEntry m_nte_ArmAngle;
  GenericEntry m_nte_ArmExtension;

  // Parameters Passed from DS via Shuffleboard
  GenericEntry m_nte_HighNodeAngle;
  GenericEntry m_nte_MidNodeAngle;
  GenericEntry m_nte_LowNodeAngle;
  GenericEntry m_nte_HighNodeExtension;
  GenericEntry m_nte_MidNodeExtension;
  GenericEntry m_nte_LowNodeExtension;

  public Arm() {
    /** Creates a new Arm. */  
    m_rotationFollower.follow(m_rotationLeader);
    m_rotationEncoder = m_rotationLeader.getEncoder();
    m_extensionEncoder = m_extensionMotor.getEncoder();

    // inst = NetworkTableInstance.getDefault();
    // m_nt = inst.getTable(ShuffleboardConstants.ArmTab);
    
    // Create and get reference to SB tab
    m_sbt_Arm = Shuffleboard.getTab(ShuffleboardConstants.ArmTab);

    // Create Widges for CURRENT Arm Position & Angle
    m_nte_ArmAngle = m_sbt_Arm.addPersistent("Current Arm Angle", getArmRotationDegrees())
          .withSize(2, 1).withPosition(0, 0).getEntry();
    m_nte_ArmExtension = m_sbt_Arm.addPersistent("Current Arm Extension", getArmExtensionInches())
          .withSize(2, 1).withPosition(0, 1).getEntry();

    // Create widgets for TARGET Arm Position
    m_nte_HighNodeExtension = m_sbt_Arm.addPersistent("High Node Extension", ArmConstants.kHighNodePosition)
          .withSize(2, 1).withPosition(2, 0).getEntry();
    m_nte_MidNodeExtension  = m_sbt_Arm.addPersistent("Mid Node Extension", ArmConstants.kMidNodePosition)
          .withSize(2, 1).withPosition(2, 1).getEntry();
    m_nte_LowNodeExtension = m_sbt_Arm.addPersistent("Low Node Extension", ArmConstants.kLowNodePosition)
          .withSize(2, 1).withPosition(2, 2).getEntry();

    // Create widgets for TARGET Arm Angle
    m_nte_HighNodeAngle     = m_sbt_Arm.addPersistent("High Node Angle", ArmConstants.kHighNodeAngle)
          .withSize(2, 1).withPosition(4, 0).getEntry();
    m_nte_MidNodeAngle  = m_sbt_Arm.addPersistent("Mid Node Angle", ArmConstants.kMidNodeAngle)
          .withSize(2, 1).withPosition(4, 1).getEntry();
    m_nte_LowNodeAngle = m_sbt_Arm.addPersistent("Low Node Angle", ArmConstants.kLowNodeAngle)
          .withSize(2, 1).withPosition(4, 2).getEntry();
              
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

  private double getArmRotationEncoder() {
    return m_rotationEncoder.getPosition();
  }

  private double getArmExtensionEncoder() {
    return m_extensionEncoder.getPosition();

  }

  // Convert Encoder ticks to inches
  private double getArmExtensionInches() {
    return getArmExtensionEncoder() * ArmConstants.kArmExtensionTicksToInches;
  }
  // Convert Encoder ticks to degrees 
  private double getArmRotationDegrees() {
    return getArmRotationEncoder() * ArmConstants.kArmRotationTicksToDegrees;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_nte_ArmAngle.setDouble(getArmRotationDegrees());
    m_nte_ArmExtension.setDouble(getArmExtensionInches());
    }

}
