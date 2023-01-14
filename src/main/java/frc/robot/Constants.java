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

    public static final class ConSparkMax {
        public static final double POSITION_CONVERSION_FACTOR = 42.0;
    }
        
    public static final class DriveConstants {
        public static final double GEAR_RATIO = 10.71; // Neo rotates 10.71 times for one rotation of the output
        public static final double WHEEL_DIAMETER_INCHES = 6.0; // Inches
        public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI; // Abt 18.85 in.
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

        public static final double kMaxSpeedMetersPerSecond = 10; //Somewhat random number right now
        public static final double kMaxAccelerationMetersPerSSquared = 25; //Also somewhat random number

        public static final double kLevel = 0; //0 degrees is level
        public static final double kBalanceToleranceDeg = 0;
        public static final double kBalanceP = 0.025;
        public static final double kBalanceI = 0;
        public static final double kBalanceD = 0;  
        
        public static final double kAutoDriveTolerance = 1;
        public static final double kAutoDriveP = 0.01;
        public static final double kAutoDriveI = 0;
        public static final double kAutoDriveD = 0;  

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
