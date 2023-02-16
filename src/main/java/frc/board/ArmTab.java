// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.board;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.constants.ArmConstants;
import frc.constants.ShuffleboardConstants;

/** Add your docs here. */
public class ArmTab {

    // Create and get reference to SB tab
    ShuffleboardTab m_sbt_Arm;

    // Encoders/PID Feedback sensors
    GenericEntry m_nte_ArmExtension;

    // Parameters Passed from DS via Shuffleboard
    GenericEntry m_nte_HighNodeExtension;
    GenericEntry m_nte_MidNodeExtension;
    GenericEntry m_nte_LowNodeExtension;

    public ArmTab() {

        // Create and get reference to SB tab
        m_sbt_Arm = Shuffleboard.getTab(ShuffleboardConstants.ArmTab);

        m_nte_ArmExtension = m_sbt_Arm.addPersistent("Current Arm Extension", getArmExtensionInches())
            .withSize(2, 1).withPosition(0, 1).getEntry();

        // Create widgets for TARGET Arm Position
        m_nte_HighNodeExtension = m_sbt_Arm.addPersistent("High Node Extension", ArmConstants.kHighNodePosition)
            .withSize(2, 1).withPosition(2, 0).getEntry();
        m_nte_MidNodeExtension  = m_sbt_Arm.addPersistent("Mid Node Extension", ArmConstants.kMidNodePosition)
            .withSize(2, 1).withPosition(2, 1).getEntry();
        m_nte_LowNodeExtension = m_sbt_Arm.addPersistent("Low Node Extension", ArmConstants.kLowNodePosition)
            .withSize(2, 1).withPosition(2, 2).getEntry();
    

    }
}
