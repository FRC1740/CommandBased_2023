// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.board;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotShared;
import frc.robot.commands.AutoBalancePID;
import frc.robot.commands.TurnToAngleProfiled;

/** Add your docs here. */
public class CommandTab {

    // Create and get reference to SB tab
    ShuffleboardTab m_sbt_Command;

    private RobotShared m_robotShared;

    private static CommandTab instance;

    private CommandTab() {
        m_robotShared = RobotShared.getInstance();
        initShuffleboardTab();
    }

    public static CommandTab getInstance() {
        if(instance == null) {
            instance = new CommandTab();
        }
        return instance;
    }

    private void initShuffleboardTab() {
        // Create and get reference to SB tab
        m_sbt_Command = Shuffleboard.getTab("Commands");

        ShuffleboardLayout commandLayout = m_sbt_Command
            .getLayout("Testable Commands", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(1, 1)
            .withProperties(Map.of("Number of columns", "2", "Label position", "LEFT"));
            
        commandLayout.add("Auto Balance", 
            new AutoBalancePID(m_robotShared.getDriveSubsystem()));

        commandLayout.add("Turn 90 degrees", 
            new TurnToAngleProfiled(90, m_robotShared.getDriveSubsystem()));
    }

}
