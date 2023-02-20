// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.constants.ShuffleboardConstants;

/** Add your docs here. */
public class ArmTable {

    NetworkTable m_nt;

    private static ArmTable instance = null;

    private ArmTable() {
        initNetworkTableInstance();
    }

    public static ArmTable getInstance() {
        if(instance == null) {
            instance = new ArmTable();
        }
        return instance;
    }

    private void initNetworkTableInstance() {
        m_nt = NetworkTableInstance.getDefault().getTable(ShuffleboardConstants.ArmTab);
    }

}
