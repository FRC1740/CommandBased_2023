// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.board;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.constants.ShuffleboardConstants;
import frc.robot.Robot;
import frc.robot.RobotProperties;

/** Add your docs here. */
public class RobotTab {

    Shuffleboard m_sb;

    // Shuffleboard DriveTrain entries
    // Create and get reference to SB tab
    ShuffleboardTab m_sbt_Robot;

    // Encoders/PID Feedback sensors
    GenericEntry m_nte_BuildTime;
    GenericEntry m_nte_BuildComputer;

    private Robot m_robot;

    public RobotTab(Robot robot) {
        m_robot = robot;
        init();
    }

    private void init() {
        // Create and get reference to SB tab

        m_sbt_Robot = Shuffleboard.getTab(ShuffleboardConstants.RobotTab);
        // See https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/shuffleboard/layouts-with-code/using-tabs.html

        // Create widget for code version

        m_nte_BuildTime = m_sbt_Robot.addPersistent("Build Time", RobotProperties.getTime())
            .withSize(3, 1).withPosition(0, 0).getEntry();

        m_nte_BuildComputer = m_sbt_Robot.addPersistent("Build Computer", RobotProperties.getComputer())
            .withSize(3, 1).withPosition(0, 1).getEntry();
    }
}
