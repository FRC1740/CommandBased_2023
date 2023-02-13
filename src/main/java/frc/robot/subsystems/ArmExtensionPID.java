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

public class ArmExtensionPID extends PIDSubsystem {

  private final CANSparkMax m_extensionMotor = new CANSparkMax(ArmConstants.kExtensionMotorPort, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder m_extensionEncoder;

  // Create and get reference to SB tab
  ShuffleboardTab m_sbt_Arm;

  // Encoders/PID Feedback sensors
  GenericEntry m_nte_ArmExtension;

  // Parameters Passed from DS via Shuffleboard
  GenericEntry m_nte_HighNodeExtension;
  GenericEntry m_nte_MidNodeExtension;
  GenericEntry m_nte_LowNodeExtension;

  /** Creates a new Telescope. */
  public ArmExtensionPID() {
    super(
        // The PIDController used by the subsystem
        new PIDController(.01, 0, 0));

    m_extensionEncoder = m_extensionMotor.getEncoder();
    m_extensionEncoder.setPosition(ArmConstants.kStowedPosition);
    m_extensionEncoder.setPositionConversionFactor(ArmConstants.ARM_EXTENSION_POSITION_CONVERSION_FACTOR);

    m_extensionMotor.burnFlash();
    
    // Initial setpoint for starting configuration (stowed, 0.0)
    setSetpoint(ArmConstants.kStowedPosition);
    
    // Create and get reference to SB tab
    m_sbt_Arm = Shuffleboard.getTab(ShuffleboardConstants.ArmTab);
    
    m_nte_ArmExtension = m_sbt_Arm.addPersistent("Current Arm Extension", getArmExtensionInches())
    .withSize(2, 1).withPosition(0, 1).getEntry();

    // Create widgets for TARGET Arm Position
    m_nte_HighNodeExtension = m_sbt_Arm.addPersistent("High Node Extension", ArmConstants.kHighNodePosition)
          .withSize(2, 1).withPosition(2, 0).getEntry();
    m_nte_MidNodeExtension  = m_sbt_Arm.addPersistent("Mid Node Extension", ArmConstants.kMidNodePosition)
          .withSize(2, 1).withPosition(2, 1).getEntry();
    m_nte_LowNodeExtension = m_sbt_Arm.addPersistent("Low Node Extension", ArmConstants.kLowNodePosition)
          .withSize(2, 1).withPosition(2, 2).getEntry();
  }

  @Override
  public void periodic() {
    super.periodic();
    m_nte_ArmExtension.setDouble(getArmExtensionInches());
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    // setpoint may be useful for a feedforward adjustment
    m_extensionMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_extensionEncoder.getPosition();
  }
  public double getArmExtensionInches() {
    return m_extensionEncoder.getPosition();
  }

  public void telescope(double speed) {
      m_extensionMotor.set(speed);
      return;
  }

}
