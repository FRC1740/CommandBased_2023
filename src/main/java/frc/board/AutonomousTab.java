// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.board;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.ShuffleboardConstants;
import frc.constants.OIConstants.GamePiece;
import frc.robot.commands.auto.*;
import frc.robot.commands.driver.ToggleGamePiece;

/** Add your docs here. */
public class AutonomousTab {

    private ShuffleboardTab m_sbt_Autonomous;
    
    private SendableChooser<Command> m_AutoChooser;
    
    private GenericEntry m_nte_GamePieceName;

    private static AutonomousTab instance;

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
        m_sbt_Autonomous = Shuffleboard.getTab(ShuffleboardConstants.AutonomousTabTab);

        m_AutoChooser = new SendableChooser<Command>();
        m_AutoChooser.addOption("RB_1", new RB_1());
        m_AutoChooser.addOption("RB_1_McDouble", new RB_1_McDouble());
        m_AutoChooser.addOption("RB_2_Exit_Balance", new RB_2_Exit_Balance_Vision());
        m_AutoChooser.addOption("RB_2_Cube_Balance", new RB_2_Cube_Balance());
        m_AutoChooser.addOption("RB_3", new RB_3());
        m_AutoChooser.addOption("RB_3_McDouble", new RB_3_McDouble());
        m_AutoChooser.addOption("Cable_Protector_McDouble", new Blue_3_McDouble());
        m_AutoChooser.addOption("Substation_McDouble", new Blue_1_McDouble());
        m_AutoChooser.addOption("Cube Balance", new Cube_Balance());
        
    
        m_sbt_Autonomous.add("Autonomous Command", m_AutoChooser)
            .withSize(2, 1)
            .withPosition(0, 0);

        m_nte_GamePieceName = m_sbt_Autonomous
            .add("Current Game Piece", "")
            .withSize(2, 1)
            .withPosition(3, 0)
            .getEntry();

        m_sbt_Autonomous.add("Toggle Game Piece", new ToggleGamePiece())
            .withSize(2, 1)
            .withPosition(3, 1);
        
    }

    public Command getAutonomousCommand() {
        return m_AutoChooser.getSelected();
    }

    public void setGamePiece(GamePiece gamePiece) {
        m_nte_GamePieceName.setString(gamePiece.name());
    }
}
