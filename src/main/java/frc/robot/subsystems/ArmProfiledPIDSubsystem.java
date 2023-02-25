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
import frc.constants.ArmTunable;
import frc.board.ArmTab;

public class ArmProfiledPIDSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new ProfiledPIDArmSubsystem. */
  private final ArmTab m_ArmTab;
  private final CANSparkMax m_rotationLeader = new CANSparkMax(ArmConstants.kRotationLeaderMotorPort, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rotationFollower = new CANSparkMax(ArmConstants.kRotationFollowerMotorPort, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder m_rotationEncoder;
  private final RelativeEncoder m_rotationFollowerEncoder;
  private final ArmFeedforward m_ArmFeedforward;
  private final ProfiledPIDController m_PIDController; 
  
  public ArmProfiledPIDSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
          ArmTunable.getRotateP(), ArmTunable.getRotateI(), ArmTunable.getRotateD(),
            // The motion profile constraints
            new TrapezoidProfile.Constraints(ArmTunable.rotateMaxVelocity, ArmTunable.rotateMaxAcceleration)));

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

        setGoal(ArmConstants.kStowedAngle);

        m_ArmTab = ArmTab.getInstance();
        m_PIDController = getController();
  }

  @Override
  public void periodic(){
    super.periodic();
    m_ArmTab.setArmAngle(getArmRotationDegrees());
  //   m_PIDController.setPID(m_ArmTab.getRotkP(),
  //                         m_ArmTab.getRotkI(), 
  //                         m_ArmTab.getRotkD());
   }
  
  private double getArmRotationDegrees() {  
    return getMeasurement();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here

    // Calculate the feedforward from the setpoint. 
    // Note that sysid says to use horizontal as the reference position
    //double feedforward = m_ArmFeedforward.calculate(setpoint.position - ArmConstants.ArmRotationAngleOffset, setpoint.velocity);
    // FIXME: After PID tuning, add the feedforward back in
    double feedforward = 0;

    // Add the feedforward to the PID output to get the motor output
    m_rotationLeader.setVoltage(output + feedforward);
  }

  public void resetToEncoderToStowedAngle(){
    m_rotationEncoder.setPosition(ArmConstants.kStowedAngle);
    m_rotationFollowerEncoder.setPosition(ArmConstants.kStowedAngle);
  }

  public void manualArmRotate(double speed){
    double position = getArmRotationDegrees();
    disable();
    if ((speed > 0.0 && position < ArmConstants.kArmRotateMaxDegrees) ||
        (speed < 0.0 && position > ArmConstants.kArmRotateMinDegrees)) {
      m_rotationLeader.set(speed);
    }
    else {
      m_rotationLeader.set(0.0);
    }
  }

  public void manualDone(){
    enable();
    setGoal(m_rotationEncoder.getPosition());
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_rotationEncoder.getPosition();
  }

  public void burnFlash() {
    m_rotationLeader.burnFlash();
    m_rotationFollower.burnFlash();
  }

}
