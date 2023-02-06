package frc.constants;

import edu.wpi.first.wpilibj.Preferences;

public class ArmTunable {

    private static final String namespace = "ArmTunable_";

    private static final String rotatePKey = namespace + "Rotate_P";
    private static final String rotateIKey = namespace + "Rotate_I";
    private static final String rotateDKey = namespace + "Rotate_D";

    private static final double rotatePDefault = 0.01;
    private static final double rotateIDefault = 0.0;
    private static final double rotateDDefault = 0.0;

    private static double getDoubleValue(String key, double defaultValue) {
        if(!Preferences.containsKey(key)) {
            Preferences.setDouble(key, defaultValue);
        }
        return Preferences.getDouble(key, defaultValue);
    }

    public static double getRotateP() {
        return getDoubleValue(rotatePKey, rotatePDefault);
    }

    public static double getRotateI() {
        return getDoubleValue(rotateIKey, rotateIDefault);
    }

    public static double getRotateD() {
        return getDoubleValue(rotateDKey, rotateDDefault);
    }

}
