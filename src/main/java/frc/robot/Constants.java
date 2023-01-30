// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final class ConMath {
//        public static final double PI = M_PI; // use: import java.lang.Math.*
        public static final double INCHES_TO_METERS = .0254; // m/in
        public static final double MINUTES_TO_SECONDS = 1/60.; // sec/min
        public static final double RAD_TO_DEG = 180.0/Math.PI;
        public static final double DEG_TO_RAD = 1/RAD_TO_DEG;
    }

    public static final class ConSignalLed {
        public static class gamePiece {
            int red, green, blue;

            public gamePiece(int r, int g, int b) {
                red = r;
                green = g;
                blue = b;
            }
            public int getRed() {
                return red;
            }
            public int getGreen() {
                return green;
            }
            public int getBlue() {
                return blue;
            }
        }
    }
    public static final class ConSparkMax {

    }
    
    public static final class DriveConstants {

    
    }
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kCoDriverControllerPort = 1;
    }
    
}
