// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.board.ArmTab;
import frc.constants.ArmConstants;

public class TelescopePIDSubsystem extends PIDSubsystem {

  private final CANSparkMax m_extensionMotor = new CANSparkMax(ArmConstants.kExtensionMotorPort, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder m_extensionEncoder;
  private ArmTab m_ArmTab;

  /** Creates a new Telescope. */
  public TelescopePIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(
          ArmConstants.extendPDefault,
          ArmConstants.extendIDefault,
          ArmConstants.extendDDefault));

    m_extensionEncoder = m_extensionMotor.getEncoder();
    m_extensionEncoder.setPosition(ArmConstants.kStowedPosition);
    m_extensionEncoder.setPositionConversionFactor(ArmConstants.ARM_EXTENSION_POSITION_CONVERSION_FACTOR);
    
    setSoftLimits(
      ArmConstants.kMinSoftLimitPosition,
      ArmConstants.kMaxSoftLimitPosition);
      
    // Initial setpoint for starting configuration (stowed, 0.0)
    setSetpoint(ArmConstants.kStowedPosition);
    
    m_ArmTab = ArmTab.getInstance();
    m_ArmTab.setArmExtension(getArmExtensionInches());
  }

  private void setSoftLimits(float minLimit, float maxLimit) {
    m_extensionMotor.setSoftLimit(SoftLimitDirection.kForward, maxLimit);
    m_extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, minLimit);
    m_extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  @Override
  public void periodic() {
    super.periodic();
    m_ArmTab.setArmExtension(getMeasurement());
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
    return getMeasurement();
  }

  public void telescope(double speed) {
      m_extensionMotor.set(speed);
      return;
  }

  public void manualTelescope(double analogInput) {
    updateManualLimits();
    double adjustedSpeed = analogInput * ArmConstants.kArmExtendInputMultiplier;
    disable();
    if (Math.abs(analogInput) > ArmConstants.kArmExtendDeadzone) {
      m_extensionMotor.set(adjustedSpeed);
    } else {
      m_extensionMotor.set(0.0);
    }
  }
  
  private void updateManualLimits() {
    if(m_ArmTab.getArmAngle() <= ArmConstants.kArmRotateLowerDegrees) {
      setSoftLimits(
        ArmConstants.kArmExtendMinInches,
        ArmConstants.kArmExtendUpperMaxInches);
    }
    else {
      setSoftLimits(
        ArmConstants.kArmExtendMinInches,
        ArmConstants.kArmExtendLowerMaxInches);
    }
  }

  public void manualDone() {
    enable();
    setSoftLimits(ArmConstants.kMinSoftLimitPosition, ArmConstants.kMaxSoftLimitPosition);
    setSetpoint(m_extensionEncoder.getPosition());
  }

  // Returns true when telescope is at setpoint
  public boolean atSetpoint() {
    return (Math.abs(m_extensionEncoder.getPosition() - getSetpoint()) < ArmConstants.kArmExtendTolerance);
  }

  public void burnFlash() {
    m_extensionMotor.burnFlash();
  }

}
