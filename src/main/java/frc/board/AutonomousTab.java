// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.board;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.CurvyPath;
import frc.robot.commands.auto.MoreCurvyPath;
import frc.robot.commands.auto.ShortStraightPath;
import frc.robot.commands.auto.StraightPath;

/** Add your docs here. */
public class AutonomousTab {

    SendableChooser<Command> m_AutoChooser = new SendableChooser<>();
    
    private static AutonomousTab instance;

    // auto command
    // Note: The autocommand is dynamically retrieved in getAutonomousCommand.
    // This member variable is currently unused
    // private Command m_autoCommand = m_AutoChooser.getSelected();

    private AutonomousTab() {
        initShuffleboardTab();
    }

    public static AutonomousTab getInstance() {
        if(instance == null) {
            instance = new AutonomousTab();
        }
        return instance;
    }

    private void initShuffleboardTab() {
        m_AutoChooser.addOption("curvy path", new CurvyPath());
        m_AutoChooser.addOption("straight", new StraightPath());
        m_AutoChooser.addOption("Short Straight path", new ShortStraightPath());
        m_AutoChooser.addOption("more curvy path", new MoreCurvyPath());
    
        Shuffleboard.getTab("Autonomous").add(m_AutoChooser);
    }

    public Command getAutonomousCommand() {
        return m_AutoChooser.getSelected();
    }

}
