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
    public static final double kArmExtensionGearRatio = 9; //Gear box is 5x3, Sprockets 1/1 //Changing to 12:1 3/13/23
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
    public static final double kStartingPosition      = 0.0;
    public static final double kStowedAngle           = 5.0;
    public static final float kMinSoftLimitAngle      = 0;
    public static final float kMaxSoftLimitAngle      = 173;
    public static final double kStowedPosition        = 1.0;
    public static final float kMinSoftLimitPosition   = 0;
    public static final float kMaxSoftLimitPosition   = 30;

    public static final double kExtensionPowerScalar = 0.7;

    public enum AutoMode {
        HIGH,
        MID,
        LOW,
        SHELF,
        FLOOR,
        STOWED
    }    

    // Cone
    public static final double kConeHighAngle         = 82; // Was 77: Too high in practice match
    public static final double kConeMidAngle          = 78;
    public static final double kConeLowAngle          = 140;
    public static final double kConeShelfAngle        = 87;
    public static final double kConeFloorAngle        = 165;
    public static final double kConeDunkAngle         = 5;

    public static final double kConeHighPosition      = 27.5;
    public static final double kConeMidPosition       = 8.5;
    public static final double kConeLowPosition       = 0;
    public static final double kConeShelfPosition     = 7.5;
    public static final double kConeFloorPosition     = 7;

    // Cube
    public static final double kCubeHighAngle         = 89;
    public static final double kCubeMidAngle          = 95;
    public static final double kCubeLowAngle          = 142;
    public static final double kCubeShelfAngle        = 87;
    public static final double kCubeFloorAngle        = 170;
    public static final double kCubeDunkAngle         = 0;

    public static final double kCubeHighPosition      = 19.0;
    public static final double kCubeMidPosition       = 0;
    public static final double kCubeLowPosition       = 0;
    public static final double kCubeShelfPosition     = 7.5;
    public static final double kCubeFloorPosition     = 0;


    public static final double rotatePDefault = 0.12149;
    public static final double rotateIDefault = 0.0;
    public static final double rotateDDefault = 0.039139*2*0;

    public static final double extendPDefault = 0.16; // Was .08. Tweaked for more speed
    public static final double extendIDefault = 0.002;//0.005
    public static final double extendDDefault = 0.0;

    public static final double rotateMaxAcceleration = 800;
    public static final double rotateMaxVelocity = 800;

    public static final double kArmRotateDeadzone = 0.1;

    public static final double kArmExtendDeadzone = 0.1;

    public static final double kArmExtendTolerance = 1;
    public static final double kArmRotateTolerance = 5;

    // Arm Auto Delays
    public static final double kAutoArmScoreConeDelay = 1.5;
    public static final double kAutoArmScoreCubeDelay = 1.5;
            
    public static final double kArmRotateRelativeConeDelay = 0;
    public static final double kArmRotateRelativeCubeDelay = 0;

    public static final double kDunkScoreConeDelay = 0.5;
    public static final double kDunkScoreCubeDelay = 0.5;

    public static final double kArmStowDelay = 0.5;

}
