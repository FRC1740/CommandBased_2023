package frc.constants;


public class ArmConstants {
    
    public static final int kRotationLeaderMotorPort = 6; //Right side of the arm
    public static final int kRotationFollowerMotorPort = 7; //Left side of the arm
    public static final int kExtensionMotorPort = 8;

    // Arm Rotation Constants
    public static final double kArmRotationGearRatio = 36*(64/22); // Gear boxes 4x3x3, Sprockets 64/22 total 4x3x3x(64/22)= 144
    public static final double ARM_ROTATION_POSITION_CONVERSION_FACTOR = 360/kArmRotationGearRatio; //Encoder output in degrees the arm rotates
    public static final double ArmRotationKs = 0.14167;
    public static final double ArmRotationKv = 0.03583;
    public static final double ArmRotationKa = 0.0010212;
    public static final double ArmRotationKg = 0.24227;
    public static final double ArmRotationAngleOffset = 86.042;
    // Arm Extension Constants
    public static final double kArmExtensionGearRatio = 15; //Gear box is 5x3, Sprockets 1/1
    // One rotation of the output = 5 inches of extension
    public static final double kArmExtensionOutputToInches = 5;
    // Encoder output to inches of extension
    public static final double ARM_EXTENSION_POSITION_CONVERSION_FACTOR = kArmExtensionOutputToInches/kArmExtensionGearRatio;

    // Manual limits and speeds
    public static final double kArmExtendMaxInches = 20.0;
    public static final double kArmExtendMinInches = 0.0;
    public static final double kArmExtendManualSpeed = 0.1;

    public static final double kArmRotateMaxDegrees = 129.0;
    public static final double kArmRotateMinDegrees = 0.0;
    public static final double kArmRotateManualSpeed = 0.1;

    //public static final double kArmExtensionRotationToLinearDistance = 1.5;
    // Output rotation in 25-1 system: 25 rotations of the motor = 1 rotation of output
    //public static final double kArmExtensionTicksPerRotationOutput = ConSparkMax.POSITION_CONVERSION_FACTOR * kArmExtensionGearRatio;
    //public static final double kArmExtensionTicksToInches = kArmExtensionTicksPerRotationOutput / kArmExtensionRotationToLinearDistance;

    /*
        * All Angles based on Horizontal = 0
        * Starting Configuration: 112 deg (0); Fully Retracted
        * Mid Node Scoring: 41 deg (71); 4"  Extension
        * High Node Scoring: 37 deg (75); 28" Extension
        * Low Node Scoring: -22 deg (134); 4" Extension
        * Human player maybe same as mid-node (close)
        */
    public static final double kStowedAngle = 0;
    public static final double kHighNodeAngle = 66; // FIXME: Started at 71; Remounted w/ hardstop
    public static final double kMidNodeAngle = 70;  // Started: 75
    public static final double kLowNodeAngle = 129; // Started 134
    public static final double kSubStationAngle = 70; // Same as Mid-node? [Started 75]
    
    public static final int kStowedPosition = 0;    // FIXME: Pseudo-wild guess at node distance
    public static final int kHighNodePosition = 4;  // These values came from CAD and will likely
    public static final int kMidNodePosition = 20;  // change once the arm is installed on the robot
    public static final int kLowNodePosition = 8;  
    public static final int kSubStationPosition = 20; 

    // // ARM Rotation PID constants
    // public static final double kRotP = .05;
    // public static final double kRotI = 0.01;
    // public static final double kRotD = 0.005;

    // // ARM Extension PID constants
    // public static final double kExtP = .02;
    // public static final double kExtI = 0;
    // public static final double kExtD = 0;
    
}
