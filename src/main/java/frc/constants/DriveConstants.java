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

}
