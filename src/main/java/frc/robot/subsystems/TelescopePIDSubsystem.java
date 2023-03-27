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
    m_extensionEncoder.setPosition(ArmConstants.kStartingPosition);
    m_extensionEncoder.setPositionConversionFactor(ArmConstants.ARM_EXTENSION_POSITION_CONVERSION_FACTOR);
    
    m_extensionMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kMaxSoftLimitPosition);
    m_extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kMinSoftLimitPosition);
    m_extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // Initial setpoint for starting configuration (stowed, 0.0)
    getController().setTolerance(ArmConstants.kArmExtendTolerance);
    setSetpoint(ArmConstants.kStowedPosition);
    
    m_ArmTab = ArmTab.getInstance();
    m_ArmTab.setArmExtension(getArmExtensionInches());
  }

  @Override
  public void periodic() {
    super.periodic();
    getController().setPID(
      m_ArmTab.getExtkP(), 
      m_ArmTab.getExtkI(),
      m_ArmTab.getExtkD());

    m_ArmTab.setArmExtension(getMeasurement());
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    // setpoint may be useful for a feedforward adjustment
    m_extensionMotor.set(output * 0.7);
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
    double adjustedSpeed = analogInput * ArmConstants.kArmExtendInputMultiplier;
    disable();
    if (Math.abs(analogInput) > ArmConstants.kArmExtendDeadzone) {
      m_extensionMotor.set(adjustedSpeed);
    } else {
      m_extensionMotor.set(0.0);
    }
  }
  
  public void manualDone() {
    enable();
    setSetpoint(m_extensionEncoder.getPosition());
  }

  // Returns true when telescope is at setpoint
  // public boolean atSetpoint() {
  //   return (Math.abs(m_extensionEncoder.getPosition() - getSetpoint()) < ArmConstants.kArmExtendTolerance);
  // }
  public boolean atSetpoint(){
    return getController().atSetpoint();
  }

  public void burnFlash() {
    m_extensionMotor.burnFlash();
  }

}
