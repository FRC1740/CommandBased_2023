package frc.constants;
import edu.wpi.first.math.util.Units;

public final class ClawConstants {
    public static final int kPneumaticPortA = 6;
    public static final int kPneumaticPortB = 7;
    public static final int IntakeMotorCANID = 6;

    public static final double kPieceRecognitionDistanceInches = 1;
    public static final double kPieceRecognitionDistanceMm = 100;
    
    // Cube
    public static final double InjectCubeHighSpeed   = 1.0; // Cube intake tested at full
    public static final double InjectCubeLowSpeed    = 0.4; //positive is intake, negative is eject
    public static final double EjectCubeLowSpeed     = -0.2;

    public static final double EjectCubeManualSpeed  = -1.0;
    public static final double InjectCubeManualSpeed = -EjectCubeManualSpeed;
    
    //Cone
    public static final double InjectConeSpeed = 0.3;
    
    
    public static final double ShutdownDelay = 2.0;
    public static final int IntakePeakCurrentLimit = 35; // Amps
    public static final int IntakePeakDurationLimit = 200; // mSec
    public static final int IntakeContinuousCurrentLimit = 20; // Amps
}
