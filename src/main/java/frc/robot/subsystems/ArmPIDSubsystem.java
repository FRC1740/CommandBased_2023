// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.constants.ArmConstants;
import frc.constants.ShuffleboardConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.*;

public class ArmPIDSubsystem extends PIDSubsystem {
  private final CANSparkMax m_rotationLeader = new CANSparkMax(ArmConstants.kRotationLeaderMotorPort, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rotationFollower = new CANSparkMax(ArmConstants.kRotationFollowerMotorPort, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder m_rotationEncoder;
  private final RelativeEncoder m_rotationFollowerEncoder;
  //protected double m_setpoint;

  // Create and get reference to SB tab
  ShuffleboardTab m_sbt_Arm;

  // Encoders/PID Feedback sensors
  GenericEntry m_nte_ArmAngle;

  // Parameters Passed from DS via Shuffleboard
  GenericEntry m_nte_HighNodeAngle;
  GenericEntry m_nte_MidNodeAngle;
  GenericEntry m_nte_LowNodeAngle;

  /** Creates a new Arm. */
  public ArmPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(.005, 0, 0));

        // The target angle for PID rotation control
    // Follower motor direction is inverted
    m_rotationFollower.follow(m_rotationLeader, true);
    m_rotationEncoder = m_rotationLeader.getEncoder();
    m_rotationFollowerEncoder = m_rotationFollower.getEncoder();
    // Reset encoders to Zero position for starting configuration
    m_rotationEncoder.setPosition(ArmConstants.kStowedAngle);
    m_rotationFollowerEncoder.setPosition(ArmConstants.kStowedAngle);

    m_rotationEncoder.setPositionConversionFactor(ArmConstants.ARM_ROTATION_POSITION_CONVERSION_FACTOR);
    m_rotationFollowerEncoder.setPositionConversionFactor(ArmConstants.ARM_ROTATION_POSITION_CONVERSION_FACTOR);
    m_rotationLeader.burnFlash(); // Do we need to do this?
    m_rotationFollower.burnFlash();

    // Initial setpoint for starting configuration (stowed, 0.0)
    setSetpoint(ArmConstants.kStowedAngle);

    // Create and get reference to SB tab
    m_sbt_Arm = Shuffleboard.getTab(ShuffleboardConstants.ArmTab);

    // Create Widges for CURRENT Arm Position & Angle
    m_nte_ArmAngle = m_sbt_Arm.addPersistent("Current Arm Angle", getArmRotationDegrees())
          .withSize(2, 1).withPosition(0, 0).getEntry();

    // Create widgets for TARGET Arm Angle
    m_nte_HighNodeAngle     = m_sbt_Arm.addPersistent("High Node Angle", ArmConstants.kHighNodeAngle)
          .withSize(2, 1).withPosition(4, 0).getEntry();
    m_nte_MidNodeAngle  = m_sbt_Arm.addPersistent("Mid Node Angle", ArmConstants.kMidNodeAngle)
          .withSize(2, 1).withPosition(4, 1).getEntry();
    m_nte_LowNodeAngle = m_sbt_Arm.addPersistent("Low Node Angle", ArmConstants.kLowNodeAngle)
          .withSize(2, 1).withPosition(4, 2).getEntry();

  }

  @Override
  public void periodic() {
    // WPILib Docs say to call the parent periodic method or PID will not work
    super.periodic();
    m_nte_ArmAngle.setDouble(getArmRotationDegrees());
  }
  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    // setpoint may be useful for a feedforward adjustment
    m_rotationLeader.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_rotationEncoder.getPosition();
  }

  @Override
  public void setSetpoint(double angle) {
      System.out.println("Setting Setpoint to "+angle);
      super.setSetpoint(angle);
  }

  private double getArmRotationDegrees() {
    return getMeasurement();
  }

}
