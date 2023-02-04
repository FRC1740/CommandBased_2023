// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Comment to force commit
// Using "import static an.enum.or.constants.inner.class.*;" helps reduce verbosity
// this replaces "DoubleSolenoid.Value.kForward" with just kForward
// further reading is available at https://www.geeksforgeeks.org/static-import-java/
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.ShuffleboardConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.*;
import frc.constants.GroundIntakeConstants;

public class GroundIntake extends SubsystemBase {
  private final CANSparkMax m_intakeMotor = new CANSparkMax(GroundIntakeConstants.kIntakeMotorPort, CANSparkMax.MotorType.kBrushless);
  private final DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GroundIntakeConstants.kPneumaticPortA, GroundIntakeConstants.kPneumaticPortB);
  private final RelativeEncoder m_intakeEncoder;
  private ShuffleboardTab m_sbt_GroundIntake;
  private GenericEntry m_nte_IntakeSpeed;
  private GenericEntry m_nte_IntakeSetSpeed;
  private double m_intakeSetSpeed = 0.0;

  /** Creates a new GroundIntake. */
  public GroundIntake() {
    m_intakeEncoder = m_intakeMotor.getEncoder();
    // Create and get reference to SB tab
    m_sbt_GroundIntake = Shuffleboard.getTab(ShuffleboardConstants.ClawTab);

    // Create Widges for CURRENT Arm Position & Angle
    m_nte_IntakeSetSpeed = m_sbt_GroundIntake.addPersistent("Intake Target Speed", m_intakeSetSpeed)
          .withSize(2, 1).withPosition(0, 0).getEntry();
    m_nte_IntakeSpeed = m_sbt_GroundIntake.addPersistent("Intake Output Velocity", getIntakeVelocity())
          .withSize(2, 1).withPosition(0, 1).getEntry();
  }

  public void deploy() {
    m_intakeSolenoid.set(kForward); // FIXME: May have to swap pneumatics orientation
    setIntakeSpeed(m_intakeSetSpeed);
  }

  public void stow() {
    m_intakeSolenoid.set(kReverse); // FIXME: May have to swap pneumatics orientation
    stopIntake();
  }

  public double getIntakeVelocity() {
    return m_intakeEncoder.getVelocity(); // GroundIntakeConstants.kIntakeGearRatio;
  }

  public void setIntakeSpeed(double speed) {
    m_intakeSetSpeed = speed;
    m_intakeMotor.set(m_intakeSetSpeed);
  }

  public void stopIntake() {
    m_intakeMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Report the actual speed to the shuffleboard
    m_nte_IntakeSpeed.setDouble(getIntakeVelocity());
    m_intakeSetSpeed = m_nte_IntakeSetSpeed.getDouble(GroundIntakeConstants.kDefaultIntakeSpeed);
  }
}
