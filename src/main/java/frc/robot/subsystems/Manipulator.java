// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// Using "import static an.enum.or.constants.inner.class.*;" helps reduce verbosity
// this replaces "DoubleSolenoid.Value.kForward" with just kForward
// further reading is available at https://www.geeksforgeeks.org/static-import-java/
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Manipulator extends SubsystemBase {
  DoubleSolenoid Intake;  

  /** Creates a new Manipulator. */
  public Manipulator() {
    Intake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  }

  public void Extend() {
    Intake.set(kForward);
  }

  public void Retract() {
    Intake.set(kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
