// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.board;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.constants.ShuffleboardConstants;
import frc.constants.OIConstants.GamePiece;
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
    GenericEntry m_nte_GamePiece;

    private static RobotTab instance = null;

    private RobotTab() {
        initShuffleboardTab();
    }

    public static RobotTab getInstance() {
        if(instance == null) {
            instance = new RobotTab();
        }
        return instance;
    }

    private void initShuffleboardTab() {
        // Create and get reference to SB tab

        m_sbt_Robot = Shuffleboard.getTab(ShuffleboardConstants.RobotTab);
        // See https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/shuffleboard/layouts-with-code/using-tabs.html

        // Create widget for code version

        m_nte_BuildTime = m_sbt_Robot.addPersistent("Build Time", RobotProperties.getTime())
            .withSize(3, 1).withPosition(0, 0).getEntry();

        m_nte_BuildComputer = m_sbt_Robot.addPersistent("Build Computer", RobotProperties.getComputer())
            .withSize(3, 1).withPosition(0, 1).getEntry();

        m_nte_GamePiece = m_sbt_Robot
            .addPersistent("Game Piece", true)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(4, 1)
            .getEntry();
    }

    public String getBuildTime() {
        return m_nte_BuildTime.getString(RobotProperties.getTime());
    }

    public void setBuildTime(String time) {
        m_nte_BuildTime.setString(time);
    }

    public String getBuildComputer() {
        return m_nte_BuildComputer.getString(RobotProperties.getComputer());
    }    

    public void setBuildComputer(String computer) {
        m_nte_BuildComputer.setString(computer);
    }
    
    public GamePiece getGamePiece() {
        if(m_nte_GamePiece.getBoolean(true)) {
            return GamePiece.CUBE;
        }
        return GamePiece.CONE;
    }    

    public void setGamePiece(GamePiece piece) {
        m_nte_GamePiece.setBoolean(piece.equals(GamePiece.CUBE));
    }
    
    public GamePiece toggleGamePiece() {
        m_nte_GamePiece.setBoolean(!m_nte_GamePiece.getBoolean(true));
        return getGamePiece();
    }
}
