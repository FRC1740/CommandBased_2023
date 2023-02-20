// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.constants.ShuffleboardConstants;

/** Add your docs here. */
public class RobotTable {

    NetworkTable m_nt;

    private static RobotTable instance = null;

    private RobotTable() {
        initNetworkTableInstance();
    }

    public static RobotTable getInstance() {
        if(instance == null) {
            instance = new RobotTable();
        }
        return instance;
    }

    private void initNetworkTableInstance() {
        m_nt = NetworkTableInstance.getDefault().getTable(ShuffleboardConstants.RobotTab);
    }

}
