// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.board;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.constants.ShuffleboardConstants;

/** Add your docs here. */
public class GroundIntakeTab {

    private ShuffleboardTab m_sbt_GroundIntake;
    
    private GenericEntry m_nte_IntakeSpeed;
    private GenericEntry m_nte_IntakeSetSpeed;

    private static GroundIntakeTab instance = null;

    private GroundIntakeTab() {
        initShuffleboardTab();
    }

    public static GroundIntakeTab getInstance() {
        if(instance == null) {
            instance = new GroundIntakeTab();
        }
        return instance;
    }

    private void initShuffleboardTab() {
        // Create and get reference to SB tab
        m_sbt_GroundIntake = Shuffleboard.getTab(ShuffleboardConstants.ClawTab);

        // Create Widges for CURRENT Arm Position & Angle
        m_nte_IntakeSetSpeed = m_sbt_GroundIntake.addPersistent("Intake Target Speed", 0.0)
            .withSize(2, 1).withPosition(0, 0).getEntry();
        m_nte_IntakeSpeed = m_sbt_GroundIntake.addPersistent("Intake Output Velocity", 0.0)
            .withSize(2, 1).withPosition(0, 1).getEntry();
    }

    public Double getIntakeSpeed() {
        return m_nte_IntakeSpeed.getDouble(0.0);
    }

    public void setIntakeSpeed(Double value) {
        m_nte_IntakeSpeed.setDouble(value);
    }

    public Double getIntakeSetSpeed() {
        return m_nte_IntakeSetSpeed.getDouble(0.0);
    }

    public void setIntakeSetSpeed(Double value) {
        m_nte_IntakeSetSpeed.setDouble(value);
    }

}
