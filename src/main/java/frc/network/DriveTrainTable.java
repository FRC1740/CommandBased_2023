// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.constants.ShuffleboardConstants;

/** Add your docs here. */
public class DriveTrainTable {

    NetworkTable m_nt;

    private static DriveTrainTable instance = null;

    private DriveTrainTable() {
        initNetworkTableInstance();
    }

    public static DriveTrainTable getInstance() {
        if(instance == null) {
            instance = new DriveTrainTable();
        }
        return instance;
    }

    private void initNetworkTableInstance() {
        m_nt = NetworkTableInstance.getDefault().getTable(ShuffleboardConstants.DriveTrainTab);

        // get a topic from a NetworkTableInstance
        // the topic name in this case is the full name
        //DoubleTopic dblTopic = inst.getDoubleTopic("/drivetrain/gyro");

        // get a topic from a NetworkTable
        // the topic name in this case is the name within the table;
        // this line and the one above reference the same topic
        // DoubleTopic dtGyro = m_nt.getDoubleTopic("gyro");

        // get a type-specific topic from a generic Topic
        // Topic genericTopic = inst.getTopic("/datatable/X");
        // DoubleTopic dblTopic = new DoubleTopic(genericTopic);
    }

}
