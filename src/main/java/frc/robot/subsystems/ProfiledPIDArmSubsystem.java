// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.constants.ArmConstants;

public class ProfiledPIDArmSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new ProfiledPIDArmSubsystem. */

  private final CANSparkMax m_rotationLeader = new CANSparkMax(ArmConstants.kRotationLeaderMotorPort, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rotationFollower = new CANSparkMax(ArmConstants.kRotationFollowerMotorPort, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder m_rotationEncoder;
  private final RelativeEncoder m_rotationFollowerEncoder;
  private final ArmFeedforward m_ArmFeedforward;
  
  public ProfiledPIDArmSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
          0.8,
            0.0,
            0.0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(30, 16)));

         m_ArmFeedforward = new ArmFeedforward(ArmConstants.ArmRotationKs, ArmConstants.ArmRotationKg, ArmConstants.ArmRotationKv, ArmConstants.ArmRotationKa);

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

        setGoal(ArmConstants.kStowedAngle);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    // Calculate the feedforward from the sepoint
    double feedforward = m_ArmFeedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_rotationLeader.setVoltage(output + feedforward);

  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_rotationEncoder.getPosition();
  }
}
