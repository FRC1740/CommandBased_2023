package frc.constants;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units; // Only needed for ConSparkMaX

public class DriveConstants {

    public static final double GEAR_RATIO = 8.45; // Neo rotates 8.45 times for one rotation of the output
    public static final double WHEEL_DIAMETER_INCHES = 6.0; // Inches
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI; // Abt 18.85 in.
    public static final double DRIVE_POSITION_CONVERSION_FACTOR = Units.inchesToMeters(WHEEL_CIRCUMFERENCE_INCHES)/GEAR_RATIO; //Encoder output in meters the robot travels

    public static final double VELOCITY_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR/60; //encoders return velocity in meters per second

    //public static final double TICKS_PER_WHEEL_REVOLUTION = ConSparkMax.DRIVE_POSITION_CONVERSION_FACTOR;
    //public static final double TICKS_PER_INCH = TICKS_PER_WHEEL_REVOLUTION / WHEEL_CIRCUMFERENCE_INCHES;
    //public static final double INCHES_PER_TICK = WHEEL_CIRCUMFERENCE_INCHES / TICKS_PER_WHEEL_REVOLUTION;

    public static final int kLeftMotor1Port = 3;
    public static final int kLeftMotor2Port = 5;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 4;

    /* CRE Copied from Gyro Drive Example  */
    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterInches = 6;

    // Assumes the encoders are directly mounted on the wheel shafts
    public static final double kEncoderDistancePerPulse = DRIVE_POSITION_CONVERSION_FACTOR / (double)kEncoderCPR; 

    //sysid constants
    public static final double ks = 0.20375; //Value for testbed
    public static final double kv = 2.9751; //Value for testbed
    public static final double ka = 4.4502; //Value for testbed
    public static final double kPDriveVel = 3.0748; //Value for testbed

    public static final double kTrackWidthMeters = Units.inchesToMeters(18); //22in. for testbot, 18 in. for actual robot
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final boolean kGyroReversed = true;

    // Used for Arcade driver input modifications (Kyle's option 2)
    // Decreases rotation sensitivity at low speeds
    public static final double kRotationDeadzone      = 0.2;
    public static final double kRotationVelocityLow   = kRotationDeadzone;
    public static final double kRotationVelocityHigh  = 0.70;
    public static final double kRotationBoostLow      = 0.40;
    public static final double kRotationBoostHigh     = 1.0;

}
