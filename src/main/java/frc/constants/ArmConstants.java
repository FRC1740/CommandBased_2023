package frc.constants;

public class ArmConstants {
    
    public static final int kRotationLeaderMotorPort = 6;
    public static final int kRotationFollowerMotorPort = 7;
    public static final int kExtensionMotorPort = 8;

    /*
        * All Angles based on Horizontal = 0
        * Starting Configuration: 112 deg (0); Fully Retracted
        * Mid Node Scoring: 41 deg (71); 4"  Extension
        * High Node Scoring: 37 deg (75); 28" Extension
        * Low Node Scoring: -22 deg (134); 4" Extension
        * Human player maybe same as mid-node (close)
        */
    public static final int kStowedAngle = 0;
    public static final int kHighNodeAngle = 71; // WAG
    public static final int kMidNodeAngle = 75; // WAG
    public static final int kLowNodeAngle = 134; // WAG
    public static final int kSubStationAngle = 75; // WAG

    public static final int kStowedPosition = 0;
    public static final int kHighNodePosition = 4; // WAG
    public static final int kMidNodePosition = 28; // WAG
    public static final int kLowNodePostion = 4; // WAG 
    public static final int kSubStationPosition = 75; // WAG
        
}
