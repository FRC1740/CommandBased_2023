// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.board.ArmTab;
import frc.constants.ArmConstants;
import frc.constants.ArmTunable;
import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmPIDSubsystem extends PIDSubsystem {
  private final CANSparkMax m_rotationLeader = new CANSparkMax(ArmConstants.kRotationLeaderMotorPort, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rotationFollower = new CANSparkMax(ArmConstants.kRotationFollowerMotorPort, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder m_rotationEncoder;
  private final RelativeEncoder m_rotationFollowerEncoder;
  private final ArmFeedforward m_ArmFeedforward;
  //protected double m_setpoint;
  private ArmTab m_ArmTab;

  /** Creates a new Arm. */
  public ArmPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ArmTunable.getRotateP(), ArmTunable.getRotateI(), ArmTunable.getRotateD()));

    m_ArmFeedforward = new ArmFeedforward(ArmConstants.ArmRotationKs, ArmConstants.ArmRotationKg, ArmConstants.ArmRotationKv, ArmConstants.ArmRotationKa);
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

    // Initial setpoint for starting configuration (stowed, 0.0)
    setSetpoint(ArmConstants.kStowedAngle);

    m_ArmTab = ArmTab.getInstance();
    m_ArmTab.setArmAngle(getArmRotationDegrees());
  }

  @Override
  public void periodic() {
    // WPILib Docs say to call the parent periodic method or PID will not work
    super.periodic();
    m_ArmTab.setArmAngle(getArmRotationDegrees());
  }
  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    // setpoint may be useful for a feedforward adjustment
    double feedforward = m_ArmFeedforward.calculate(setpoint,0);
    m_rotationLeader.setVoltage(output + m_ArmFeedforward.calculate(setpoint, 0));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_rotationEncoder.getPosition();
  }

  @Override
  public void setSetpoint(double angle) {
      // System.out.println("Setting Setpoint to "+angle);
      super.setSetpoint(angle);
  }

  private double getArmRotationDegrees() {
    return getMeasurement();
  }

  public void burnFlash() {
    m_rotationLeader.burnFlash();
    m_rotationFollower.burnFlash();
  }
  
}
