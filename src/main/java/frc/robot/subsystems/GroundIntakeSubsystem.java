// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.constants.OIConstants;

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
import frc.board.GroundIntakeTab;
import frc.constants.GroundIntakeConstants;

public class GroundIntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_intakeMotor = new CANSparkMax(GroundIntakeConstants.kIntakeMotorPort, CANSparkMax.MotorType.kBrushless);
  private final DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, GroundIntakeConstants.kPneumaticPortA, GroundIntakeConstants.kPneumaticPortB);
  private final RelativeEncoder m_intakeEncoder;
  private double m_intakeSetSpeed = GroundIntakeConstants.kDefaultIntakeSpeed;

  private GroundIntakeTab m_GroundIntakeTab;

  private OIConstants.GamePiece m_gamePiece = OIConstants.kDefaultGamePiece;

  /** Creates a new GroundIntake. */
  public GroundIntakeSubsystem() {
    m_intakeEncoder = m_intakeMotor.getEncoder();

    m_GroundIntakeTab = GroundIntakeTab.getInstance();
    m_GroundIntakeTab.setIntakeSetSpeed(m_intakeSetSpeed);
    m_GroundIntakeTab.setIntakeSpeed(getIntakeVelocity());

    m_gamePiece = OIConstants.kDefaultGamePiece;
}

  // Gamepiece Cone/Cube is a global mode
  // FIXME: GroundIntake has state based on Gamepiece and task. First create the state machine for each GamePiece; only then implement

  public void deploy() {
    m_intakeSolenoid.set(kForward); // FIXME: May have to swap pneumatics orientation
    setIntakeSpeed(m_intakeSetSpeed);
  }

  public void stow() {
    m_intakeSolenoid.set(kReverse); // FIXME: May have to swap pneumatics orientation
    stopIntake();
  }

  public void eject() {
    setIntakeSpeed(-m_intakeSetSpeed);
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
    m_GroundIntakeTab.setIntakeSpeed(getIntakeVelocity());
    m_intakeSetSpeed = m_GroundIntakeTab.getIntakeSetSpeed();
  }

  public void setGamePiece(OIConstants.GamePiece gamePiece) {
    m_gamePiece = gamePiece;
  }

  public void burnFlash() {
    m_intakeMotor.burnFlash();
  }

}
