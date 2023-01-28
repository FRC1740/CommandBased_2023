package frc.constants;

public final class ClawConstants {
    public static final int kPneumaticPortA = 0;
    public static final int kPneumaticPortB = 1;
    public static final int IntakeMotorCANID = 9;
    public static final double InjectCubeHighSpeed = .6; // FIXME: WAG cube intake high speed
    public static final double InjectCubeLowSpeed = .1; // FIXME: WAG cube intake low speed (hold)
    public static final double EjectCubeSpeed = -InjectCubeHighSpeed;
    public static final double InjectConeSpeed = .5; // FIXME: WAG cone intake speed
    public static final double ShutdownDelay = 1.0;
    public static final int IntakePeakCurrentLimit = 35; // Amps
    public static final int IntakePeakDurationLimit = 200; // mSec
    public static final int IntakeContinuousCurrentLimit = 20; // Amps
}
