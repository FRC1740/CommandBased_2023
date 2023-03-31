// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.DriveConstants;
import frc.constants.GroundIntakeConstants;
import frc.robot.commands.basic.IntakeDeploy;
import frc.robot.commands.basic.IntakeEject;
import frc.robot.commands.basic.IntakeStow;

/** Add your docs here. */
public class Paths {

    //Auto Paths
    public PathPlannerTrajectory Blue_3_McDouble;
    public PathPlannerTrajectory Blue_3_McTriple;
    public PathPlannerTrajectory Blue_1_McTriple;
    public PathPlannerTrajectory Blue_1_McDouble;
    public PathPlannerTrajectory Blue_3_McDouble_Combo_Meal;
    public PathPlannerTrajectory Blue_3_McDouble_Deluxe_pt1;
    public PathPlannerTrajectory Blue_3_McDouble_Deluxe_pt2;
    public PathPlannerTrajectory Blue_3_McDouble_Deluxe_pt3;


    //Event map
    public static HashMap<String, Command> eventMap = new HashMap<>();
    
    Paths(){
    
    Blue_1_McDouble = 
    PathPlanner.loadPath("Blue 1 McDouble", new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared), true);

    Blue_3_McDouble = 
    PathPlanner.loadPath("Blue 3 McDouble", new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared), true);

    Blue_3_McTriple = 
    PathPlanner.loadPath("Blue 3 McTriple", new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared), true);

    Blue_1_McTriple =
    PathPlanner.loadPath("Blue 1 McTriple", new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared), true);

    Blue_3_McDouble_Combo_Meal =
    PathPlanner.loadPath("Blue 3 McDouble Combo Meal", new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared), true);

    Blue_3_McDouble_Deluxe_pt1 = 
    PathPlanner.loadPath("Blue 3 McDouble Deluxe pt1", new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared), true);

    Blue_3_McDouble_Deluxe_pt2 = 
    PathPlanner.loadPath("Blue 3 McDouble Deluxe pt2", new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared), false);

    Blue_3_McDouble_Deluxe_pt3 = 
    PathPlanner.loadPath("Blue 3 McDouble Deluxe pt3", new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared), false);

    

        eventMap.put("Intake Deploy", new IntakeDeploy(GroundIntakeConstants.kCubeIntakeSpeed));
        eventMap.put("Intake Stow", new IntakeStow());
        eventMap.put("Eject Cube Low Speed", new IntakeEject(GroundIntakeConstants.kCubeEjectSpeedLow));
        eventMap.put("Eject Cube High Speed", new IntakeEject(GroundIntakeConstants.kCubeEjectSpeed));
    }

    public static HashMap<String, Command> getEventMap(){
        return eventMap;
    }
}