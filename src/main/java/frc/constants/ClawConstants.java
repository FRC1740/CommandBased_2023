package frc.constants;

public final class ClawConstants {
    public static final int kPneumaticPortA = 6;
    public static final int kPneumaticPortB = 7;
    public static final int IntakeMotorCANID = 6;

    public static final double kPieceRecognitionDistanceMmMax = 200;
    public static final double kPieceRecognitionDistanceMmMin = 100;
    
    // Cube
    public static final double kCubeInjectHighSpeed   = 1.0; // Cube intake tested at full
    public static final double kCubeInjectLowSpeed    = 0.6; // positive is intake, negative is eject
    public static final double kCubeEjectLowSpeed     =-0.3;

    // Manual
    public static final double kManualEjectSpeed  = -1.0;
    public static final double kManualInjectSpeed = 0.2;
    
    //Cone
    public static final double kConeInjectSpeed = 1.0;
    
    
    public static final double ShutdownDelay = 2.0;
    public static final int IntakePeakCurrentLimit = 35; // Amps
    public static final int IntakePeakDurationLimit = 200; // mSec
    public static final int IntakeContinuousCurrentLimit = 20; // Amps
}
