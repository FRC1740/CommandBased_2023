package frc.constants;

public final class ClawConstants {
    public static final int kPneumaticPortA = 6;
    public static final int kPneumaticPortB = 7;
    public static final int IntakeMotorCANID = 6;

    // Cube
    public static final double InjectCubeHighSpeed   = 1.0; // Cube intake tested at full
    public static final double InjectCubeLowSpeed    = 0.5;
    public static final double EjectCubeLowSpeed     = -InjectCubeLowSpeed;

    public static final double EjectCubeManualSpeed  = 1.0;
    public static final double InjectCubeManualSpeed = -EjectCubeManualSpeed;
    
    //Cone
    public static final double InjectConeSpeed = 0.0;
    
    
    public static final double ShutdownDelay = 1.0;
    public static final int IntakePeakCurrentLimit = 35; // Amps
    public static final int IntakePeakDurationLimit = 200; // mSec
    public static final int IntakeContinuousCurrentLimit = 20; // Amps
}
