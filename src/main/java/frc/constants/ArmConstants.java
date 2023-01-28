package frc.constants;
import frc.robot.Constants.ConSparkMax;

public class ArmConstants {
    
    public static final int kRotationLeaderMotorPort = 6;
    public static final int kRotationFollowerMotorPort = 7;
    public static final int kExtensionMotorPort = 8;

    // Arm Rotation Constants
    public static final double kArmRotationGearRatio = 144; // Gear boxes 4x3x3, Sprockets 64/16 = 4 total 4x3x3x4 = 144
    public static final double kArmRotationTicksToDegrees = ConSparkMax.POSITION_CONVERSION_FACTOR * kArmRotationGearRatio / 360;
    // Arm Extension Constants
    public static final double kArmExtensionGearRatio = 25; // FIXME: Extension Gear ratio may change
    // One rotation of the output = 1.5 inches of extension
    public static final double kArmExtensionRotationToLinearDistance = 1.5;
    // Output rotation in 25-1 system: 25 rotations of the motor = 1 rotation of output
    public static final double kArmExtensionTicksPerRotationOutput = ConSparkMax.POSITION_CONVERSION_FACTOR * kArmExtensionGearRatio;
    public static final double kArmExtensionTicksToInches = kArmExtensionTicksPerRotationOutput / kArmExtensionRotationToLinearDistance;

    /*
        * All Angles based on Horizontal = 0
        * Starting Configuration: 112 deg (0); Fully Retracted
        * Mid Node Scoring: 41 deg (71); 4"  Extension
        * High Node Scoring: 37 deg (75); 28" Extension
        * Low Node Scoring: -22 deg (134); 4" Extension
        * Human player maybe same as mid-node (close)
        */
    public static final int kStowedAngle = 0;
    public static final int kHighNodeAngle = 71; // FIXME: Pseudo-wild guess at node angle
    public static final int kMidNodeAngle = 75;  // These values came from CAD and will likely
    public static final int kLowNodeAngle = 134; // change once the arm is installed on the robot
    public static final int kSubStationAngle = 75; 

    public static final int kStowedPosition = 0;    // FIXME: Pseudo-wild guess at node distance
    public static final int kHighNodePosition = 4;  // These values came from CAD and will likely
    public static final int kMidNodePosition = 28;  // change once the arm is installed on the robot
    public static final int kLowNodePosition = 4;  
    public static final int kSubStationPosition = 75; 
        
}
