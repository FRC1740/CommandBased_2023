// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.board;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.constants.ArmConstants;
import frc.constants.ArmTunable;
import frc.constants.ShuffleboardConstants;

/** Add your docs here. */
public class ArmTab {

    // Create and get reference to SB tab
    ShuffleboardTab m_sbt_Arm;
    
    // Encoders/PID Feedback sensors
    GenericEntry m_nte_ArmExtension;
    GenericEntry m_nte_ArmAngle;
    
    // Parameters Passed from DS via Shuffleboard
    GenericEntry m_nte_HighNodeAngle;
    GenericEntry m_nte_MidNodeAngle;
    GenericEntry m_nte_LowNodeAngle;
    GenericEntry m_nte_HighNodeExtension;
    GenericEntry m_nte_MidNodeExtension;
    GenericEntry m_nte_LowNodeExtension;

    // Arm Rotation & Extension PID settings
    GenericEntry m_nte_kPRot;
    GenericEntry m_nte_kIRot;
    GenericEntry m_nte_kDRot;
    GenericEntry m_nte_kPExt;
    GenericEntry m_nte_kIExt;
    GenericEntry m_nte_kDExt;
    
    private static ArmTab instance = null;

    private ArmTab() {
        initShuffleboardTab();
    }

    public static ArmTab getInstance() {
        if(instance == null) {
            instance = new ArmTab();
        }
        return instance;
    }

    private void initShuffleboardTab() {
        // Create and get reference to SB tab
        m_sbt_Arm = Shuffleboard.getTab(ShuffleboardConstants.ArmTab);

        m_nte_ArmExtension = m_sbt_Arm.addPersistent("Current Arm Extension", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(3, 3).withPosition(5, 0).getEntry();
        m_nte_ArmAngle = m_sbt_Arm.addPersistent("Current Arm Angle", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(3, 3).withPosition(0, 0).getEntry();
  
        // Create widgets for TARGET Arm Position
        m_nte_HighNodeExtension = m_sbt_Arm.addPersistent("High Node Extension", ArmConstants.kHighNodePosition)
            .withSize(1, 1).withPosition(8, 0).getEntry();
        m_nte_MidNodeExtension  = m_sbt_Arm.addPersistent("Mid Node Extension", ArmConstants.kMidNodePosition)
            .withSize(1, 1).withPosition(8, 1).getEntry();
        m_nte_LowNodeExtension = m_sbt_Arm.addPersistent("Low Node Extension", ArmConstants.kLowNodePosition)
            .withSize(1, 1).withPosition(8, 2).getEntry();

        // Create widgets for TARGET Arm Angle
        m_nte_HighNodeAngle = m_sbt_Arm.addPersistent("High Node Angle", ArmConstants.kHighNodeAngle)
            .withSize(1, 1).withPosition(3, 0).getEntry();
        m_nte_MidNodeAngle = m_sbt_Arm.addPersistent("Mid Node Angle", ArmConstants.kMidNodeAngle)
            .withSize(1, 1).withPosition(3, 1).getEntry();
        m_nte_LowNodeAngle = m_sbt_Arm.addPersistent("Low Node Angle", ArmConstants.kLowNodeAngle)
            .withSize(1, 1).withPosition(3, 2).getEntry();

        // Create widgets for Arm Angle PID
        m_nte_kPRot = m_sbt_Arm.addPersistent("Rot kP", ArmTunable.getRotateP())
            .withSize(1, 1).withPosition(4, 0).getEntry();
        m_nte_kIRot = m_sbt_Arm.addPersistent("Rot kI", ArmTunable.getRotateI())
            .withSize(1, 1).withPosition(4, 1).getEntry();
        m_nte_kDRot = m_sbt_Arm.addPersistent("Rot kD", ArmTunable.getRotateD())
            .withSize(1, 1).withPosition(4, 2).getEntry();

        // Create widgets for Arm Telescope PID
        m_nte_kPExt = m_sbt_Arm.addPersistent("Ext kP", ArmTunable.getExtendP())
            .withSize(1, 1).withPosition(9, 0).getEntry();
        m_nte_kIExt = m_sbt_Arm.addPersistent("Ext kI", ArmTunable.getExtendI())
            .withSize(1, 1).withPosition(9, 1).getEntry();
        m_nte_kDExt = m_sbt_Arm.addPersistent("Ext kD", ArmTunable.getExtendD())
            .withSize(1, 1).withPosition(9, 2).getEntry();
            
    }

    public Double getRotkP() {
        return m_nte_kPRot.getDouble(ArmTunable.getRotateP());
    }
    public Double getRotkI() {
        return m_nte_kIRot.getDouble(ArmTunable.getRotateI());
    }
    public Double getRotkD() {
        return m_nte_kDRot.getDouble(ArmTunable.getRotateD());
    }

    public Double getExtkP() {
        return m_nte_kPExt.getDouble(ArmTunable.getExtendP());
    }
    public Double getExtkI() {
        return m_nte_kIExt.getDouble(ArmTunable.getExtendI());
    }
    public Double getExtkD() {
        return m_nte_kDExt.getDouble(ArmTunable.getExtendD());
    }


    public Double getArmExtension() {
        return m_nte_ArmExtension.getDouble(0.0);
    }

    public void setArmExtension(Double value) {
        m_nte_ArmExtension.setDouble(value);
    }
  
    public Double getArmAngle() {
        return m_nte_ArmAngle.getDouble(0.0);
    }

    public void setArmAngle(Double value) {
        m_nte_ArmAngle.setDouble(value);
    }
  
    public Double getHighNodeExtension() {
        return m_nte_HighNodeExtension.getDouble(ArmConstants.kHighNodePosition);
    }

    public void setHighNodeExtension(Double value) {
        m_nte_HighNodeExtension.setDouble(value);
    }

    public Double getMidNodeExtension() {
        return m_nte_MidNodeExtension.getDouble(ArmConstants.kMidNodePosition);
    }

    public void setMidNodeExtension(Double value) {
        m_nte_MidNodeExtension.setDouble(value);
    }

    public Double getLowNodeExtension() {
        return m_nte_LowNodeExtension.getDouble(ArmConstants.kLowNodePosition);
    }

    public void setLowNodeExtension(Double value) {
        m_nte_LowNodeExtension.setDouble(value);
    }

    public Double getHighNodeAngle() {
        return m_nte_HighNodeAngle.getDouble(ArmConstants.kHighNodeAngle);
    }

    public void setHighNodeAngle(Double value) {
        m_nte_HighNodeAngle.setDouble(value);
    }

    public Double getMidNodeAngle() {
        return m_nte_MidNodeAngle.getDouble(ArmConstants.kMidNodeAngle);
    }

    public void setMidNodeAngle(Double value) {
        m_nte_MidNodeAngle.setDouble(value);
    }

    public Double getLowNodeAngle() {
        return m_nte_LowNodeAngle.getDouble(ArmConstants.kLowNodeAngle);
    }

    public void setLowNodeAngle(Double value) {
        m_nte_LowNodeAngle.setDouble(value);
    }

}
