package frc.constants;

public final class ClawConstants {
    public static final int kPneumaticPortA = 0;
    public static final int kPneumaticPortB = 1;
    public static final int IntakeMotorCANID = 1;
    public static final double InjectCubeHighSpeed = 1.0; // Cube intake tested at full
    public static final double InjectCubeLowSpeed = .1; // FIXME: WAG cube intake low speed (hold)
    public static final double EjectCubeSpeed = -InjectCubeLowSpeed;
    public static final double InjectConeSpeed = 1.0; // FIXME: May need to adjust cone intake speed
    public static final double ShutdownDelay = 1.0;
    public static final int IntakePeakCurrentLimit = 35; // Amps
    public static final int IntakePeakDurationLimit = 200; // mSec
    public static final int IntakeContinuousCurrentLimit = 20; // Amps
}
