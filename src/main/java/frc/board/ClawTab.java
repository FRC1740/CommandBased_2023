// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.board;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.constants.ShuffleboardConstants;
import frc.robot.subsystems.ClawSubsystem.ClawMode;

/** Add your docs here. */
public class ClawTab {

    // Shuffleboard DriveTrain entries
    // Create and get reference to SB tab
    private ShuffleboardTab m_sbt_Claw;

    // Parameters Passed from DS via Shuffleboard
    private GenericEntry m_nte_ClawMode;
    // private GenericEntry m_nte_IntakeSpeed;
    private GenericEntry m_nte_IntakeCurrent;

    private static ClawTab instance = null;

    private ClawTab() {
        initShuffleboardTab();
    }

    public static ClawTab getInstance() {
        if(instance == null) {
            instance = new ClawTab();
        }
        return instance;
    }

    private void initShuffleboardTab() {
        // Create and get reference to SB tab
        m_sbt_Claw = Shuffleboard.getTab(ShuffleboardConstants.ClawTab);

        // Create Widges for CURRENT Arm Position & Angle
        m_nte_ClawMode = m_sbt_Claw.addPersistent("Claw Mode", ClawMode.READY.name())
            .withSize(2, 1).withPosition(0, 0).getEntry();
        // m_nte_IntakeSpeed = m_sbt_Claw.addPersistent("Intake Speed", getIntakeSpeed())
        //       .withSize(2, 1).withPosition(0, 1).getEntry();
        m_nte_IntakeCurrent = m_sbt_Claw.addPersistent("Intake Current", 0.0)
            .withSize(2, 1).withPosition(0, 2).getEntry();

    }

    public String getClawMode() {
        return m_nte_ClawMode.getString(ClawMode.READY.name());
    }

    public void setClawMode(String value) {
        m_nte_ClawMode.setString(value);
    }

    public Double getIntakeCurrent() {
        return m_nte_IntakeCurrent.getDouble(10.0);
    }

    public void setIntakeCurrent(Double value) {
        m_nte_IntakeCurrent.setDouble(value);
    }

    // public Double getIntakeSpeed() {
    //     return m_nte_IntakeSpeed.getDouble(10.0);
    // }

    // public void setIntakeSpeed(Double value) {
    //     m_nte_IntakeSpeed.setDouble(value);
    // }

}
