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
    public static final double kArmExtensionOutputToInches = Math.PI * 1.5;
    // Encoder output to inches of extension
    public static final double ARM_EXTENSION_POSITION_CONVERSION_FACTOR = kArmExtensionOutputToInches/kArmExtensionGearRatio;

    // Manual limits and speeds
    public static final double kArmExtendMaxInches    = 30.0;
    public static final double kArmExtendMinInches    = 0.0;
    public static final double kArmExtendManualSpeed  = 0.2;
    public static final double kArmExtendInputMultiplier = 0.3; //used to dampen joystick input

    public static final double kDumbAutoTelescopeSpeed = 0.2;
    public static final double kDumbAutoTelescopeDeadzone = 0.2;


    public static final double kArmRotateMaxDegrees   = 129.0;
    public static final double kArmRotateMinDegrees   = 5;
    public static final double kArmRotateManualSpeed  = 0.1;
    public static final double kArmRotateInputMultiplier = 0.2; //used to dampen joystick input

    // Auto limits and speeds
    public static final double kStowedAngle           = 5.0;
    public static final float kMinSoftLimitAngle      = 0;
    public static final float kMaxSoftLimitAngle      = 173;
    public static final double kStowedPosition        = 0.0;
    public static final float kMinSoftLimitPosition   = 0;
    public static final float kMaxSoftLimitPosition   = 30;

    // New ----------------------------------------------------------------------------------------
    // Cone
    public static final double kConeHighAngle         = 77;
    public static final double kConeMidAngle          = 75;
    public static final double kConeLowAngle          = 140;
    public static final double kConeShelfAngle        = 87;
    public static final double kConeFloorAngle        = 173;

    public static final double kConeHighPosition      = 28.5;
    public static final double kConeMidPosition       = 8.5;
    public static final double kConeLowPosition       = 0;
    public static final double kConeShelfPosition     = 7.5;
    public static final double kConeFloorPosition     = 0;

    // Cube
    public static final double kCubeHighAngle         = 80;
    public static final double kCubeMidAngle          = 75;
    public static final double kCubeLowAngle          = 140;
    public static final double kCubeShelfAngle        = 87;
    public static final double kCubeFloorAngle        = 173;

    public static final double kCubeHighPosition      = 27.5;
    public static final double kCubeMidPosition       = 8.5;
    public static final double kCubeLowPosition       = 0;
    public static final double kCubeShelfPosition     = 7.5;
    public static final double kCubeFloorPosition     = 0;

    // Temp- to be removed after further refactoring
    public static final double kHighNodeAngle        = kConeHighAngle;
    public static final double kMidNodeAngle         = kConeMidAngle;
    public static final double kLowNodeAngle         = kConeLowAngle;
    public static final double kMidRetrieveAngle     = kConeShelfAngle;
    public static final double kLowRetrieveAngle     = kConeFloorAngle;
    
    public static final double kHighNodePosition     = kConeHighPosition;
    public static final double kMidNodePosition      = kConeMidPosition;
    public static final double kLowNodePosition      = kConeLowPosition; 
    public static final double kMidRetrievePosition  = kConeShelfPosition;
    public static final double kLowRetrievePosition  = kConeFloorPosition;

    // Old ----------------------------------------------------------------------------------------
    // public static final double kHighNodeAngle        = 80;  // FIXME: tune angles
    // public static final double kMidNodeAngle         = 75;
    // public static final double kLowNodeAngle         = 140;
    // public static final double kMidRetrieveAngle     = 87;
    // public static final double kLowRetrieveAngle     = 173;
    // //Cone hi node 77 angle 28.5
    // //Cube hi node 80 angle, 27.5
    
    // public static final double kHighNodePosition     = 27.5; // FIXME: tune telescope
    // public static final double kMidNodePosition      = 8.5;
    // public static final double kLowNodePosition      = 0;  
    // public static final double kMidRetrievePosition  = 7.5; 
    // public static final double kLowRetrievePosition  = 0;
    //---------------------------------------------------------------------------------------------

    public static final double rotatePDefault = 0.12149;
    public static final double rotateIDefault = 0.0;
    public static final double rotateDDefault = 0.039139*2*0;

    public static final double extendPDefault = 0.08;
    public static final double extendIDefault = 0.002;//0.005
    public static final double extendDDefault = 0.0;

    public static final double rotateMaxAcceleration = 800;
    public static final double rotateMaxVelocity = 800;

    public static final double kArmRotateDeadzone = 0.1;

    public static final double kArmExtendDeadzone = 0.1;

    public static final double kArmExtendTolerance = 1;
    public static final double kArmRotateTolerance = 5;

}
