package frc.constants;


public class ArmConstants {
    
    public static final int kRotationLeaderMotorPort = 6; //Right side of the arm
    public static final int kRotationFollowerMotorPort = 7; //Left side of the arm
    public static final int kExtensionMotorPort = 8;

    // Arm Rotation Constants
    public static final double kArmRotationGearRatio = 36*(64/22); // Gear boxes 4x3x3, Sprockets 64/22 total 4x3x3x(64/22)= 144
    public static final double ARM_ROTATION_POSITION_CONVERSION_FACTOR = 360/kArmRotationGearRatio; //Encoder output in degrees the arm rotates
    public static final double ArmRotationKs = 0.1632;
    public static final double ArmRotationKv = 0.034513;
    public static final double ArmRotationKa = 0.0022757;
    public static final double ArmRotationKg = 0.23146;
    public static final double ArmRotationAngleOffset = 102.24;
    // Arm Extension Constants
    public static final double kArmExtensionGearRatio = 15; //Gear box is 5x3, Sprockets 1/1
    // One rotation of the output = 5 inches of extension
    public static final double kArmExtensionOutputToInches = 5; //Should be pi * 1.5
    // Encoder output to inches of extension
    public static final double ARM_EXTENSION_POSITION_CONVERSION_FACTOR = kArmExtensionOutputToInches/kArmExtensionGearRatio;

    // Manual limits and speeds
    public static final double kArmExtendMaxInches    = 30.0;
    public static final double kArmExtendMinInches    = 0.0;
    public static final double kArmExtendManualSpeed  = 0.2;
    public static final double kArmExtendInputMultiplier = 0.3; //used to dampen joystick input

    public static final double kArmRotateMaxDegrees   = 129.0;
    public static final double kArmRotateMinDegrees   = 5;
    public static final double kArmRotateManualSpeed  = 0.1;
    public static final double kArmRotateInputMultiplier = 0.2; //used to dampen joystick input
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
    public static final double kStowedAngle          = 5;
    public static final double kHighNodeAngle        = 70;  // FIXME: tune angles
    public static final double kMidNodeAngle         = 75;
    public static final double kLowNodeAngle         = 140;
    public static final double kMidRetrieveAngle     = 81;
    public static final double kLowRetrieveAngle     = 173;

    
    public static final double kStowedPosition       = 0;
    public static final double kHighNodePosition     = 30; // FIXME: tune telescope
    public static final double kMidNodePosition      = 9;
    public static final double kLowNodePosition      = 0;  
    public static final double kMidRetrievePosition  = 8; 
    public static final double kLowRetrievePosition  = 0; 
    
    public static final double rotatePDefault = 0.12149;
    public static final double rotateIDefault = 0.0;
    public static final double rotateDDefault = 0.039139*2*0;

    public static final double extendPDefault = .02;
    public static final double extendIDefault = 0;
    public static final double extendDDefault = 0;

    public static final double rotateMaxAcceleration = 800;
    public static final double rotateMaxVelocity = 800;   

}
