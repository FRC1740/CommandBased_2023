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
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 3;
        public static final int kLeftMotor2Port = 5;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 4;

        /* CRE Copied from Gyro Drive Example  */
        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
    
        public static final boolean kGyroReversed = false;
    
        public static final double kStabilizationP = 1;
        public static final double kStabilizationI = 0.5;
        public static final double kStabilizationD = 0;
        //Less than .5 not .25 more than .01 .08 too high .0605 dead on
        public static final double kTurnP = 0.0363;
        public static final double kTurnI = 0.201;
        public static final double kTurnD = 0.0027;
    
        public static final double kMaxTurnRateDegPerS = 100;
        public static final double kMaxTurnAccelerationDegPerSSquared = 300;
    
        public static final double kTurnToleranceDeg = 5;
        public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
    
    }
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kCoDriverControllerPort = 1;
    }
    
    /* Shouldn't need these...
    public static final class XboxController {
        public static final int kLeftXAxis = 0;
        public static final int kLeftYAxis = 1;
        public static final int kLeftTrigger = 2;
        public static final int kRightTrigger = 3;
        public static final int kRightXAxis = 4;
        public static final int kRightYAxis = 5;
    } /* */
}
