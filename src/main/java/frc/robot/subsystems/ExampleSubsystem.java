// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Relay;

public class ExampleSubsystem extends SubsystemBase {
  private final Relay m_relay;
  boolean state = false;
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    m_relay = new Relay(0);   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void toggle() {
    state = !state;
    if (state) {
      m_relay.set(Relay.Value.kForward);
    }
    else {
      m_relay.set(Relay.Value.kOff);      
    }
  }
}
