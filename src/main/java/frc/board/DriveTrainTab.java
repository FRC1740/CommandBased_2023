// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.board;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.constants.AutoConstants;
import frc.constants.ShuffleboardConstants;

/** Add your docs here. */
public class DriveTrainTab {
    Field2d m_Field = new Field2d();
    
    Shuffleboard m_sb;

    // Shuffleboard DriveTrain entries
    // Create and get reference to SB tab
    ShuffleboardTab m_sbt_DriveTrain;

    GenericEntry m_nte_Testing;

    // Autonomous Variables
    GenericEntry m_nte_a_DriveDelay;
    GenericEntry m_nte_b_DriveDistance;
    GenericEntry m_nte_c_DriveTurnAngle;
    GenericEntry m_nte_autoDriveMode;

    // Encoders/PID Feedback sensors
    GenericEntry m_nte_LeftEncoder;
    GenericEntry m_nte_RightEncoder;
    GenericEntry m_nte_IMU_ZAngle;
    GenericEntry m_nte_IMU_PitchAngle; // USES IMU ROLL AXIS!!!

    // PID Tuning
    GenericEntry m_nte_DriveSpeedFilter;
    GenericEntry m_nte_DriveRotationFilter;

    GenericEntry m_nte_kPAutoBalance;
    GenericEntry m_nte_kIAutoBalance;
    GenericEntry m_nte_kDAutoBalance;

    GenericEntry m_nte_kPTurn;
    GenericEntry m_nte_kITurn;
    GenericEntry m_nte_kDTurn;

    // Create widget for non-linear input
    GenericEntry m_nte_InputExponent;

    private static DriveTrainTab instance = null;

    private DriveTrainTab() {
        initShuffleboardTab();
        
        // Comment the next line to persist the values on the robot across deploys and restarts
        resetToDefault();
    }

    public static DriveTrainTab getInstance() {
        if(instance == null) {
            instance = new DriveTrainTab();
        }
        return instance;
    }

    private void resetToDefault() {
        // Auto Balance PID
        m_nte_kPAutoBalance.setDouble(AutoConstants.kBalanceP);
        m_nte_kIAutoBalance.setDouble(AutoConstants.kBalanceI);
        m_nte_kDAutoBalance.setDouble(AutoConstants.kBalanceD);
    
        // Turn PID
        m_nte_kPTurn.setDouble(AutoConstants.kTurnP);
        m_nte_kITurn.setDouble(AutoConstants.kTurnI);
        m_nte_kDTurn.setDouble(AutoConstants.kTurnD);
    
    }

    private void initShuffleboardTab() {
        // Create and get reference to SB tab
        m_sbt_DriveTrain = Shuffleboard.getTab(ShuffleboardConstants.DriveTrainTab);

        // Create widgets for digital filter lengths
        m_nte_DriveSpeedFilter = m_sbt_DriveTrain.addPersistent("Drive Speed Filter", 10.0)
            .withSize(2, 1).withPosition(0, 0).getEntry();
        m_nte_DriveRotationFilter = m_sbt_DriveTrain.addPersistent("Drive Rotation Filter", 5.0)
            .withSize(2, 1).withPosition(0, 1).getEntry();
        // Create widget for non-linear input
        m_nte_InputExponent = m_sbt_DriveTrain.addPersistent("Input Exponent", 1.0)
            .withSize(1, 1).withPosition(0, 2).getEntry();

        // Create widgets for AutoDrive
        m_nte_a_DriveDelay     = m_sbt_DriveTrain.addPersistent("Drive Delay", .5)
            .withSize(1, 1).withPosition(3, 0).getEntry();
        m_nte_b_DriveDistance  = m_sbt_DriveTrain.addPersistent("Drive Distance", 48)
            .withSize(1, 1).withPosition(3, 1).getEntry();
        m_nte_c_DriveTurnAngle = m_sbt_DriveTrain.addPersistent("Turn Angle", 0.0)
            .withSize(1, 1).withPosition(3, 2).getEntry();            
        m_nte_autoDriveMode    = m_sbt_DriveTrain.addPersistent("AutoDrive Mode", 2)
            .withSize(1, 1).withPosition(3, 3).getEntry();

        //  m_nte_Testing     = m_sbt_DriveTrain.addPersistent("Testing", 0.0)       .withSize(1, 1).withPosition(3, 3).getEntry();

        // Encoder outputs
        // Display current encoder values
        m_nte_LeftEncoder = m_sbt_DriveTrain.addPersistent("Left Side Encoder", 0.0)
            .withSize(2,1).withPosition(4,0).getEntry();

        m_nte_RightEncoder = m_sbt_DriveTrain.addPersistent("Right Side Encoder", 0.0)
            .withSize(2,1).withPosition(4,1).getEntry();

        m_nte_IMU_ZAngle = m_sbt_DriveTrain.addPersistent("IMU Z-Axis Angle", 0.0)
            .withSize(2,1).withPosition(4,2).getEntry();

        m_nte_IMU_PitchAngle = m_sbt_DriveTrain.addPersistent("IMU Pitch", 0.0)
            .withSize(2,1).withPosition(4,3).getEntry();

        m_sbt_DriveTrain.add(m_Field)
            .withSize(4, 2).withPosition(5, 0);

        // Create widgets for PID Controllers
        // Auto Balance
        ShuffleboardLayout autoBalancePIDLayout = m_sbt_DriveTrain
            .getLayout("Auto Balance PID", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(8, 0)
            .withProperties(Map.of("Number of columns", "2", "Label position", "LEFT"));
            
        m_nte_kPAutoBalance = autoBalancePIDLayout
            .add("AutoBalance kP", AutoConstants.kBalanceP)
            .getEntry();
        m_nte_kIAutoBalance = autoBalancePIDLayout
            .add("AutoBalance kI", AutoConstants.kBalanceI)
            .getEntry();
        m_nte_kDAutoBalance = autoBalancePIDLayout
            .add("AutoBalance kD", AutoConstants.kBalanceD)
            .getEntry();

        // Turn Profiled
        ShuffleboardLayout turnPIDLayout = m_sbt_DriveTrain
            .getLayout("Turn PID", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(8, 2)
            .withProperties(Map.of("Number of columns", "2", "Label position", "LEFT"));
 
        m_nte_kPTurn = turnPIDLayout
            .add("Turn kP", AutoConstants.kTurnP)
            .getEntry();
        m_nte_kITurn = turnPIDLayout
            .add("Turn kI", AutoConstants.kTurnI)
            .getEntry();
        m_nte_kDTurn = turnPIDLayout
            .add("Turn kD", AutoConstants.kTurnD)
            .getEntry();

    }

    public Double getAutoBalancekP() {
        return m_nte_kPAutoBalance.getDouble(AutoConstants.kBalanceP);
    }

    public Double getAutoBalancekI() {
        return m_nte_kIAutoBalance.getDouble(AutoConstants.kBalanceI);
    }

    public Double getAutoBalancekD() {
        return m_nte_kDAutoBalance.getDouble(AutoConstants.kBalanceD);
    }

    public Double getTurnkP() {
        return m_nte_kPTurn.getDouble(AutoConstants.kTurnP);
    }

    public Double getTurnkI() {
        return m_nte_kITurn.getDouble(AutoConstants.kTurnI);
    }

    public Double getTurnkD() {
        return m_nte_kDTurn.getDouble(AutoConstants.kTurnD);
    }

    public Double getDriveSpeedFilter() {
        return m_nte_DriveSpeedFilter.getDouble(10.0);
    }

    public void setDriveSpeedFilter(Double value) {
        m_nte_DriveSpeedFilter.setDouble(value);
    }

    public Double getDriveRotationFilter() {
        return m_nte_DriveRotationFilter.getDouble(5.0);
    }

    public void setDriveRotationFilter(Double value) {
        m_nte_DriveRotationFilter.setDouble(value);
    }

    public Double getInputExponent() {
        return m_nte_InputExponent.getDouble(1.0);
    }

    public void setInputExponent(Double value) {
        m_nte_InputExponent.setDouble(value);
    }

    public Double getDriveDelay() {
        return m_nte_a_DriveDelay.getDouble(0.5);
    }

    public void setDriveDelay(Double value) {
        m_nte_a_DriveDelay.setDouble(value);
    }

    public Double getDriveDistance() {
        return m_nte_b_DriveDistance.getDouble(48);
    }

    public void setDriveDistance(Double value) {
        m_nte_b_DriveDistance.setDouble(value);
    }

    public Double getDriveTurnAngle() {
        return m_nte_c_DriveTurnAngle.getDouble(0.0);
    }

    public void setDriveTurnAngle(Double value) {
        m_nte_c_DriveTurnAngle.setDouble(value);
    }

    public Double getAutoDriveMode() {
        return m_nte_autoDriveMode.getDouble(2);
    }

    public void setAutoDriveMode(Double value) {
        m_nte_autoDriveMode.setDouble(value);
    }

    public Double getLeftEncoder() {
        return m_nte_LeftEncoder.getDouble(0.0);
    }

    public void setLeftEncoder(Double value) {
        m_nte_LeftEncoder.setDouble(value);
    }

    public Double getRightEncoder() {
        return m_nte_RightEncoder.getDouble(0.0);
    }

    public void setRightEncoder(Double value) {
        m_nte_RightEncoder.setDouble(value);
    }

    public Double getIMU_ZAngle() {
        return m_nte_IMU_ZAngle.getDouble(0.0);
    }

    public void setIMU_ZAngle(Double value) {
        m_nte_IMU_ZAngle.setDouble(value);
    }

    public Double getIMU_PitchAngle() {
        return m_nte_IMU_PitchAngle.getDouble(0.0);
    }

    public void setIMU_PitchAngle(Double value) {
        m_nte_IMU_PitchAngle.setDouble(value);
    }

    public Pose2d getRobotPose() {
        return m_Field.getRobotPose();
    }

    public void setRobotPose(Pose2d pose2d) {
        m_Field.setRobotPose(pose2d);
    }

    public void setTrajectory(Trajectory traj){
        m_Field.getObject("trajectory").setTrajectory(traj);
    }
}
