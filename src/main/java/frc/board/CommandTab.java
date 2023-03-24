// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.board;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotShared;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;

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

        initDriveLayout();
        initAutoLayout();
    }

    private void initDriveLayout() {
        ShuffleboardLayout driveCommandLayout = m_sbt_Command
            .getLayout("Drive Commands", BuiltInLayouts.kList)
            .withSize(3, 3)
            .withPosition(0, 0)
            .withProperties(Map.of("Number of columns", "2", "Label position", "LEFT"));
            
        driveCommandLayout.add("Auto Balance", 
            new AutoBalancePID(m_robotShared.getDriveSubsystem()));

        driveCommandLayout.add("Turn 90 degrees", 
            new TurnToAngle(90, m_robotShared.getDriveSubsystem()));

        driveCommandLayout.add("Drive -30", 
            new DriveToDistance(Units.inchesToMeters(-30), m_robotShared.getDriveSubsystem()));
    }

    private void initAutoLayout() {
        ShuffleboardLayout autoCommandLayout = m_sbt_Command
            .getLayout("Auto Commands", BuiltInLayouts.kList)
            .withSize(3, 3)
            .withPosition(3, 0)
            .withProperties(Map.of("Number of columns", "2", "Label position", "LEFT"));
        
        autoCommandLayout.add("Station 1", new RB_1());
        autoCommandLayout.add("Station 1 - Claw Ready", new RB_1_McDouble());
        autoCommandLayout.add("Station 2", new RB_2());
        autoCommandLayout.add("Station 2 - Exit Balance", new RB_2_Exit_Balance());
        autoCommandLayout.add("Station 2 - Pickup", new RB_2_Pickup());
        autoCommandLayout.add("Station 3", new RB_3());
        autoCommandLayout.add("Station 3 - Claw Ready", new RB_3_McDouble());
    }

}
