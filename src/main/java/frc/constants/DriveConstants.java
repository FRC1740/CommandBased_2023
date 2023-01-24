package frc.constants;
import frc.robot.Constants.*; // FIXME: Only needed for ConSparkMaX

public class DriveConstants {

    public static final double GEAR_RATIO = 10.71; // Neo rotates 10.71 times for one rotation of the output
    public static final double WHEEL_DIAMETER_INCHES = 6.0; // Inches
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI; // Abt 18.85 in.
    public static final double TICKS_PER_WHEEL_REVOLUTION = ConSparkMax.POSITION_CONVERSION_FACTOR * GEAR_RATIO;
    public static final double TICKS_PER_INCH = TICKS_PER_WHEEL_REVOLUTION / WHEEL_CIRCUMFERENCE_INCHES;
    public static final double INCHES_PER_TICK = WHEEL_CIRCUMFERENCE_INCHES / TICKS_PER_WHEEL_REVOLUTION;

    public static final int kLeftMotor1Port = 3;
    public static final int kLeftMotor2Port = 5;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 4;

    /* CRE Copied from Gyro Drive Example  */
    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterInches = 6;

    // Assumes the encoders are directly mounted on the wheel shafts
    public static final double kEncoderDistancePerPulse = kWheelDiameterInches * Math.PI / (double)kEncoderCPR;


    public static final boolean kGyroReversed = false;

    public static final double kStabilizationP = 1;
    public static final double kStabilizationI = 0.5;
    public static final double kStabilizationD = 0;
    //Less than .5 not .25 more than .01 .08 too high .0605 dead on
    public static final double kTurnP = 0.005;
    public static final double kTurnI = 0.00;
    public static final double kTurnD = 0.0;

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double kMaxSpeedMetersPerSecond = 10; //Somewhat random number right now
    public static final double kMaxAccelerationMetersPerSSquared = 25; //Also somewhat random number

    public static final double kLevel = 0; //0 degrees is level
    public static final double kBalanceToleranceDeg = 1.5;
    public static final double kBalanceP = 0.006;//0.01
    public static final double kBalanceI = 0;
    public static final double kBalanceD = 0;  
    
    public static final double kAutoDriveTolerance = 1;
    public static final double kAutoDriveP = 0.01;
    public static final double kAutoDriveI = 0;
    public static final double kAutoDriveD = 0;  

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    public static final double kDriveCorrectionP = .01; //coefficient for course correction in autoDrive commands

    public static final double kDriveToChargeStationPower = .4; //power for drive to charge station command

    public static final double kDriveToDistancePower = 0.3; //power for drive to distance command

}
