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

public class ArmExtensionPID extends PIDSubsystem {

  private final CANSparkMax m_extensionMotor = new CANSparkMax(ArmConstants.kExtensionMotorPort, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder m_extensionEncoder;
  private ArmTab m_ArmTab;

  /** Creates a new Telescope. */
  public ArmExtensionPID() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ArmConstants.kExtP, ArmConstants.kExtI, ArmConstants.kExtD));

    m_extensionEncoder = m_extensionMotor.getEncoder();
    m_extensionEncoder.setPosition(ArmConstants.kStowedPosition);
    m_extensionEncoder.setPositionConversionFactor(ArmConstants.ARM_EXTENSION_POSITION_CONVERSION_FACTOR);

    m_extensionMotor.burnFlash();
    
    // Initial setpoint for starting configuration (stowed, 0.0)
    setSetpoint(ArmConstants.kStowedPosition);
    
    m_ArmTab = ArmTab.getInstance();
    m_ArmTab.setArmExtension(getArmExtensionInches());
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

  public void manualTelescopeOut(){
    disable();
    m_extensionMotor.set(0.1);
  }

  public void manualTelescopeIn(){
    disable();
    m_extensionMotor.set(-0.1);
    
  }

  public void manualDone(){
    enable();
    setSetpoint(m_extensionEncoder.getPosition());
  }
}
