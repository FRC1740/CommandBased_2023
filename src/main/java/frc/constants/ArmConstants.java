package frc.constants;
import frc.robot.Constants.ConSparkMax;

public class ArmConstants {
    
    public static final int kRotationLeaderMotorPort = 6;
    public static final int kRotationFollowerMotorPort = 7;
    public static final int kExtensionMotorPort = 8;

    // Arm Rotation Constants
    public static final double kArmRotationGearRatio = 20; // FIXME
    public static final double kArmRotationTicksToDegrees = ConSparkMax.POSITION_CONVERSION_FACTOR / kArmRotationGearRatio / 360;

    // Arm Extension Constants
    public static final double kArmExtensionGearRatio = 20; // FIXME
    public static final double kArmExtensionOutputDiameterInches = 4; // FIXME
    public static final double kArmExtensionTicksPerRotationOutput = ConSparkMax.POSITION_CONVERSION_FACTOR / kArmExtensionGearRatio;
    public static final double kArmExtensionTicksToInches = kArmExtensionTicksPerRotationOutput * Math.PI * kArmExtensionOutputDiameterInches;

    /*
        * All Angles based on Horizontal = 0
        * Starting Configuration: 112 deg (0); Fully Retracted
        * Mid Node Scoring: 41 deg (71); 4"  Extension
        * High Node Scoring: 37 deg (75); 28" Extension
        * Low Node Scoring: -22 deg (134); 4" Extension
        * Human player maybe same as mid-node (close)
        */
    public static final int kStowedAngle = 0;
    public static final int kHighNodeAngle = 71; // FIXME
    public static final int kMidNodeAngle = 75; // FIXME
    public static final int kLowNodeAngle = 134; // FIXME
    public static final int kSubStationAngle = 75; // FIXME

    public static final int kStowedPosition = 0;
    public static final int kHighNodePosition = 4; // FIXME
    public static final int kMidNodePosition = 28; // FIXME
    public static final int kLowNodePosition = 4; // FIXME 
    public static final int kSubStationPosition = 75; // FIXME
        
}
