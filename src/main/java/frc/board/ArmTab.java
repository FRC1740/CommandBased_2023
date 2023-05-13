// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.board;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.constants.ArmConstants;
import frc.constants.ShuffleboardConstants;

/** Add your docs here. */
public class ArmTab {

    // Create and get reference to SB tab
    ShuffleboardTab m_sbt_Arm;
    
    // Encoders/PID Feedback sensors
    GenericEntry m_nte_ArmExtension;
    GenericEntry m_nte_ArmAngle;
    GenericEntry m_nte_ArmAngleSetpoint;
    
    // Cone
    GenericEntry m_nte_ConeHighAngle;
    GenericEntry m_nte_ConeMidAngle;
    GenericEntry m_nte_ConeLowAngle;
    GenericEntry m_nte_ConeShelfAngle;
    GenericEntry m_nte_ConeFloorAngle;
    GenericEntry m_nte_ConeDunkAngle;

    GenericEntry m_nte_ConeHighPosition;
    GenericEntry m_nte_ConeMidPosition;
    GenericEntry m_nte_ConeLowPosition;
    GenericEntry m_nte_ConeShelfPosition;
    GenericEntry m_nte_ConeFloorPosition;

    // Cube
    GenericEntry m_nte_CubeHighAngle;
    GenericEntry m_nte_CubeMidAngle;
    GenericEntry m_nte_CubeLowAngle;
    GenericEntry m_nte_CubeShelfAngle;
    GenericEntry m_nte_CubeFloorAngle;
    GenericEntry m_nte_CubeDunkAngle;

    GenericEntry m_nte_CubeHighPosition;
    GenericEntry m_nte_CubeMidPosition;
    GenericEntry m_nte_CubeLowPosition;
    GenericEntry m_nte_CubeShelfPosition;
    GenericEntry m_nte_CubeFloorPosition;

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
        
        // Comment the next line to persist the values on the robot across deploys and restarts
        resetToDefault();
    }

    public static ArmTab getInstance() {
        if(instance == null) {
            instance = new ArmTab();
        }
        return instance;
    }

    private void resetToDefault() {
        // Cone Angles
        m_nte_ConeHighAngle.setDouble(ArmConstants.kConeHighAngle);
        m_nte_ConeMidAngle.setDouble(ArmConstants.kConeMidAngle);
        m_nte_ConeLowAngle.setDouble(ArmConstants.kConeLowAngle);
        m_nte_ConeShelfAngle.setDouble(ArmConstants.kConeShelfAngle);
        m_nte_ConeFloorAngle.setDouble(ArmConstants.kConeFloorAngle);
        m_nte_ConeDunkAngle.setDouble(ArmConstants.kConeDunkAngle);

        // Cube Angles
        m_nte_CubeHighAngle.setDouble(ArmConstants.kCubeHighAngle);
        m_nte_CubeMidAngle.setDouble(ArmConstants.kCubeMidAngle);
        m_nte_CubeLowAngle.setDouble(ArmConstants.kCubeLowAngle);
        m_nte_CubeShelfAngle.setDouble(ArmConstants.kCubeShelfAngle);
        m_nte_CubeFloorAngle.setDouble(ArmConstants.kCubeFloorAngle);
        m_nte_CubeDunkAngle.setDouble(ArmConstants.kCubeDunkAngle);

        // Cone Extension
        m_nte_ConeHighPosition.setDouble(ArmConstants.kConeHighPosition);
        m_nte_ConeMidPosition.setDouble(ArmConstants.kConeMidPosition);
        m_nte_ConeLowPosition.setDouble(ArmConstants.kConeLowPosition);
        m_nte_ConeShelfPosition.setDouble(ArmConstants.kConeShelfPosition);
        m_nte_ConeFloorPosition.setDouble(ArmConstants.kConeFloorPosition);

        // Cube Extension
        m_nte_CubeHighPosition.setDouble(ArmConstants.kCubeHighPosition);
        m_nte_CubeMidPosition.setDouble(ArmConstants.kCubeMidPosition);
        m_nte_CubeLowPosition.setDouble(ArmConstants.kCubeLowPosition);
        m_nte_CubeShelfPosition.setDouble(ArmConstants.kCubeShelfPosition);
        m_nte_CubeFloorPosition.setDouble(ArmConstants.kCubeFloorPosition);

        // Extension PID
        m_nte_kPExt.setDouble(ArmConstants.extendPDefault);
        m_nte_kIExt.setDouble(ArmConstants.extendIDefault);
        m_nte_kDExt.setDouble(ArmConstants.extendDDefault);
            
        // Rotation PID
        m_nte_kPRot.setDouble(ArmConstants.rotatePDefault);
        m_nte_kIRot.setDouble(ArmConstants.rotateIDefault);
        m_nte_kDRot.setDouble(ArmConstants.rotateDDefault);
    }

    private void initShuffleboardTab() {
        // Create and get reference to SB tab
        m_sbt_Arm = Shuffleboard.getTab(ShuffleboardConstants.ArmTab);

        // Graph of Arm Angle
        m_nte_ArmAngle = m_sbt_Arm
            .addPersistent("Current Arm Angle", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(3, 3)
            .withPosition(0, 0)
            .getEntry();

        m_nte_ArmAngleSetpoint = m_sbt_Arm
            .addPersistent("Current Arm Angle Setpoint", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(3, 3)
            .withPosition(10, 0)
            .getEntry();

    
        // Create widgets for TARGET Arm Angle
        // Cone
        ShuffleboardLayout armAngleConeLayout = m_sbt_Arm
            .getLayout("Arm Angle Setpoints (Cone)", BuiltInLayouts.kList)
            .withSize(2, 1)
            .withPosition(3, 1)
            .withProperties(Map.of("Number of columns", "2", "Label position", "LEFT"));
            
        m_nte_ConeHighAngle = armAngleConeLayout
            .addPersistent("Cone High Angle", ArmConstants.kConeHighAngle)
            .getEntry();
        m_nte_ConeMidAngle = armAngleConeLayout
            .addPersistent("Cone Mid Angle", ArmConstants.kConeMidAngle)
            .getEntry();
        m_nte_ConeLowAngle = armAngleConeLayout
            .addPersistent("Cone Low Angle", ArmConstants.kConeLowAngle)
            .getEntry();
        m_nte_ConeShelfAngle = armAngleConeLayout
            .addPersistent("Cone Shelf Angle", ArmConstants.kConeShelfAngle)
            .getEntry();
        m_nte_ConeFloorAngle = armAngleConeLayout
            .addPersistent("Cone Floor Angle", ArmConstants.kConeFloorAngle)
            .getEntry();
        m_nte_ConeDunkAngle = armAngleConeLayout
            .addPersistent("Cone Dunk Angle", ArmConstants.kConeDunkAngle)
            .getEntry();

        // Cube
        ShuffleboardLayout armAngleCubeLayout = m_sbt_Arm
            .getLayout("Arm Angle Setpoints (Cube)", BuiltInLayouts.kList)
            .withSize(2, 1)
            .withPosition(3, 2)
            .withProperties(Map.of("Number of columns", "2", "Label position", "LEFT"));

        m_nte_CubeHighAngle = armAngleCubeLayout
            .addPersistent("Cube High Angle", ArmConstants.kCubeHighAngle)
            .getEntry();
        m_nte_CubeMidAngle = armAngleCubeLayout
            .addPersistent("Cube Mid Angle", ArmConstants.kCubeMidAngle)
            .getEntry();
        m_nte_CubeLowAngle = armAngleCubeLayout
            .addPersistent("Cube Low Angle", ArmConstants.kCubeLowAngle)
            .getEntry();
        m_nte_CubeShelfAngle = armAngleCubeLayout
            .addPersistent("Cube Shelf Angle", ArmConstants.kCubeShelfAngle)
            .getEntry();
        m_nte_CubeFloorAngle = armAngleCubeLayout
            .addPersistent("Cube Floor Angle", ArmConstants.kCubeFloorAngle)
            .getEntry();
        m_nte_CubeDunkAngle = armAngleCubeLayout
            .addPersistent("Cube Dunk Angle", ArmConstants.kCubeDunkAngle)
            .getEntry();
 
        // Graph of Arm Extension
        m_nte_ArmExtension = m_sbt_Arm
            .addPersistent("Current Arm Extension", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(3, 3)
            .withPosition(5, 0)
            .getEntry();
  
        // Create widgets for TARGET Arm Extenstion
        // Cone
        ShuffleboardLayout armExtensionConeLayout = m_sbt_Arm
            .getLayout("Arm Extension Setpoints (Cone)", BuiltInLayouts.kList)
            .withSize(2, 1)
            .withPosition(8, 1)
            .withProperties(Map.of("Number of columns", "2", "Label position", "LEFT"));

        m_nte_ConeHighPosition = armExtensionConeLayout
            .addPersistent("Cone High Position", ArmConstants.kConeHighPosition)
            .getEntry();
        m_nte_ConeMidPosition  = armExtensionConeLayout
            .addPersistent("Cone Mid Position", ArmConstants.kConeMidPosition)
            .getEntry();
        m_nte_ConeLowPosition = armExtensionConeLayout
            .addPersistent("Cone Low Position", ArmConstants.kConeLowPosition)
            .getEntry();
        m_nte_ConeShelfPosition = armExtensionConeLayout
            .addPersistent("Cone Shelf Position", ArmConstants.kConeShelfPosition)
            .getEntry();
        m_nte_ConeFloorPosition = armExtensionConeLayout
            .addPersistent("Cone Floor Position", ArmConstants.kConeFloorPosition)
            .getEntry();

        // Cube
        ShuffleboardLayout armExtensionCubeLayout = m_sbt_Arm
            .getLayout("Arm Extension Setpoints (Cube)", BuiltInLayouts.kList)
            .withSize(2, 1)
            .withPosition(8, 2)
            .withProperties(Map.of("Number of columns", "2", "Label position", "LEFT"));

        m_nte_CubeHighPosition = armExtensionCubeLayout
            .addPersistent("Cube High Position", ArmConstants.kCubeHighPosition)
            .getEntry();
        m_nte_CubeMidPosition  = armExtensionCubeLayout
            .addPersistent("Cube Mid Position", ArmConstants.kCubeMidPosition)
            .getEntry();
        m_nte_CubeLowPosition = armExtensionCubeLayout
            .addPersistent("Cube Low Position", ArmConstants.kCubeLowPosition)
            .getEntry();
        m_nte_CubeShelfPosition = armExtensionCubeLayout
            .addPersistent("Cube Shelf Position", ArmConstants.kCubeShelfPosition)
            .getEntry();
        m_nte_CubeFloorPosition = armExtensionCubeLayout
            .addPersistent("Cube Floor Position", ArmConstants.kCubeFloorPosition)
            .getEntry();

        // Create widgets for Arm Telescope PID
        ShuffleboardLayout armTelescopePIDLayout = m_sbt_Arm
            .getLayout("Arm Extension PID", BuiltInLayouts.kList)
            .withSize(2, 1)
            .withPosition(8, 0)
            .withProperties(Map.of("Number of columns", "2", "Label position", "LEFT"));

        m_nte_kPExt = armTelescopePIDLayout
            .add("Ext kP", ArmConstants.extendPDefault)
            .getEntry();
        m_nte_kIExt = armTelescopePIDLayout
            .add("Ext kI", ArmConstants.extendIDefault)
            .getEntry();
        m_nte_kDExt = armTelescopePIDLayout
            .add("Ext kD", ArmConstants.extendDDefault)
            .getEntry();
            
        // Create widgets for Arm Angle PID
        ShuffleboardLayout armRotationPIDLayout = m_sbt_Arm
            .getLayout("Arm Rotation PID", BuiltInLayouts.kList)
            .withSize(2, 1)
            .withPosition(3, 0)
            .withProperties(Map.of("Number of columns", "2", "Label position", "LEFT"));

        m_nte_kPRot = armRotationPIDLayout
            .addPersistent("Rot kP", ArmConstants.rotatePDefault)
            .getEntry();
        m_nte_kIRot = armRotationPIDLayout
            .addPersistent("Rot kI", ArmConstants.rotateIDefault)
            .getEntry();
        m_nte_kDRot = armRotationPIDLayout
            .addPersistent("Rot kD", ArmConstants.rotateDDefault)
            .getEntry();

    }

    // Getters for Arm Rotation PID
    public Double getRotkP() {
        return m_nte_kPRot.getDouble(ArmConstants.rotatePDefault);
    }

    public Double getRotkI() {
        return m_nte_kIRot.getDouble(ArmConstants.rotateIDefault);
    }

    public Double getRotkD() {
        return m_nte_kDRot.getDouble(ArmConstants.rotateDDefault);
    }

    // Getters for Arm Extension PID
    public Double getExtkP() {
        return m_nte_kPExt.getDouble(ArmConstants.extendPDefault);
    }

    public Double getExtkI() {
        return m_nte_kIExt.getDouble(ArmConstants.extendIDefault);
    }

    public Double getExtkD() {
        return m_nte_kDExt.getDouble(ArmConstants.extendDDefault);
    }

    // Getters and Setters for actual Arm Extension and Angle
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

    public void setArmAngleSetpoint(Double value){
        m_nte_ArmAngleSetpoint.setDouble(value);
    }
  
    // Getters for Arm angle setpoints
    // Cone
    public Double getConeHighAngle() {
        return m_nte_ConeHighAngle.getDouble(ArmConstants.kConeHighAngle);
    }
    
    public Double getConeMidAngle() {
        return m_nte_ConeMidAngle.getDouble(ArmConstants.kConeMidAngle);
    }
    
    public Double getConeLowAngle() {
        return m_nte_ConeLowAngle.getDouble(ArmConstants.kConeLowAngle);
    }
    
    public Double getConeShelfAngle() {
        return m_nte_ConeShelfAngle.getDouble(ArmConstants.kConeShelfAngle);
    }
    
    public Double getConeFloorAngle() {
        return m_nte_ConeFloorAngle.getDouble(ArmConstants.kConeFloorAngle);
    }
    
    public Double getConeDunkAngle() {
        return m_nte_ConeDunkAngle.getDouble(ArmConstants.kConeDunkAngle);
    }
    
    // Cube
    public Double getCubeHighAngle() {
        return m_nte_CubeHighAngle.getDouble(ArmConstants.kCubeHighAngle);
    }

    public Double getCubeMidAngle() {
        return m_nte_CubeMidAngle.getDouble(ArmConstants.kCubeMidAngle);
    }

    public Double getCubeLowAngle() {
        return m_nte_CubeLowAngle.getDouble(ArmConstants.kCubeLowAngle);
    }

    public Double getCubeShelfAngle() {
        return m_nte_CubeShelfAngle.getDouble(ArmConstants.kCubeShelfAngle);
    }

    public Double getCubeFloorAngle() {
        return m_nte_CubeFloorAngle.getDouble(ArmConstants.kCubeFloorAngle);
    }

    public Double getCubeDunkAngle() {
        return m_nte_CubeDunkAngle.getDouble(ArmConstants.kCubeDunkAngle);
    }

    // Getters for Arm position setpoints
    // Cone
    public Double getConeHighPosition() {
        return m_nte_ConeHighPosition.getDouble(ArmConstants.kConeHighPosition);
    }
    
    public Double getConeMidPosition() {
        return m_nte_ConeMidPosition.getDouble(ArmConstants.kConeMidPosition);
    }
    
    public Double getConeLowPosition() {
        return m_nte_ConeLowPosition.getDouble(ArmConstants.kConeLowPosition);
    }
    
    public Double getConeShelfPosition() {
        return m_nte_ConeShelfPosition.getDouble(ArmConstants.kConeShelfPosition);
    }
    
    public Double getConeFloorPosition() {
        return m_nte_ConeFloorPosition.getDouble(ArmConstants.kConeFloorPosition);
    }
    
    // Cube
    public Double getCubeHighPosition() {
        return m_nte_CubeHighPosition.getDouble(ArmConstants.kCubeHighPosition);
    }

    public Double getCubeMidPosition() {
        return m_nte_CubeMidPosition.getDouble(ArmConstants.kCubeMidPosition);
    }

    public Double getCubeLowPosition() {
        return m_nte_CubeLowPosition.getDouble(ArmConstants.kCubeLowPosition);
    }

    public Double getCubeShelfPosition() {
        return m_nte_CubeShelfPosition.getDouble(ArmConstants.kCubeShelfPosition);
    }

    public Double getCubeFloorPosition() {
        return m_nte_CubeFloorPosition.getDouble(ArmConstants.kCubeFloorPosition);
    }

}
