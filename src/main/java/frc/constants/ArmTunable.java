package frc.constants;

import edu.wpi.first.wpilibj.Preferences;

public class ArmTunable {

    private static final String namespace = "ArmTunable_";

    private static final String rotatePKey = namespace + "Rotate_P";
    private static final String rotateIKey = namespace + "Rotate_I";
    private static final String rotateDKey = namespace + "Rotate_D";

    private static final String extendPKey = namespace + "Extend_P";
    private static final String extendIKey = namespace + "Extend_I";
    private static final String extendDKey = namespace + "Extend_D";

    private static final double rotatePDefault = 0.12149;
    private static final double rotateIDefault = 0.0;
    private static final double rotateDDefault = 0.039139*2;

    public static final double rotateMaxAcceleration = 800;
    public static final double rotateMaxVelocity = 800;   

    public static final double extendPDefault = .02;
    public static final double extendIDefault = 0;
    public static final double extendDDefault = 0;

    private static double getDoubleValue(String key, double defaultValue) {
        if(!Preferences.containsKey(key)) {
            Preferences.setDouble(key, defaultValue);
        }
        return Preferences.getDouble(key, defaultValue);
    }

    public static double getRotateP() {
        // return getDoubleValue(rotatePKey, rotatePDefault);
        return rotatePDefault;
    }

    public static double getRotateI() {
        return getDoubleValue(rotateIKey, rotateIDefault);
    }

    public static double getRotateD() {
        return getDoubleValue(rotateDKey, rotateDDefault);
    }

    public static double getExtendP() {
        return getDoubleValue(extendPKey, extendPDefault);
    }

    public static double getExtendI() {
        return getDoubleValue(extendIKey, extendIDefault);
    }

    public static double getExtendD() {
        return getDoubleValue(extendDKey, extendDDefault);
    }
   

}
