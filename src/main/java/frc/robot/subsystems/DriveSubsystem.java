// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.ConSparkMax;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.RelativeEncoder;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new Drivetrain. */
    private static final CANSparkMax m_leftMotorLeader = new CANSparkMax(DriveConstants.kLeftMotor1Port, CANSparkMax.MotorType.kBrushless);
    private static final CANSparkMax m_rightMotorLeader = new CANSparkMax(DriveConstants.kRightMotor1Port, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax m_leftMotorFollower = new CANSparkMax(DriveConstants.kLeftMotor2Port, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax m_rightMotorFollower = new CANSparkMax(DriveConstants.kRightMotor2Port, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static final RelativeEncoder m_leftEncoder = m_leftMotorLeader.getEncoder();
    public static final RelativeEncoder m_rightEncoder = m_rightMotorLeader.getEncoder();
    // gyro NavX IMU CRASHIN
    private static final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotorLeader, m_rightMotorLeader);
    public DriveSubsystem() {
       m_leftMotorFollower.follow(m_leftMotorLeader);
       m_rightMotorFollower.follow(m_rightMotorLeader);  
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  
  public void arcadeDrive(double fwd, double rot, boolean squaredInput) {
    m_drive.arcadeDrive(fwd, rot, squaredInput);
  }

  /**
  * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
  *
  * @param maxOutput the maximum output to which the drive will be constrained
  */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getAngle() {
    System.out.println("gyro angle" + m_gyro.getAngle());
    return m_gyro.getAngle();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public double getRoll(){
    return m_gyro.getRoll();
  }
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double inchesToEncoderTicks(double inches) {
    //Converts Inches into Encoder ticks
    double encoderTicks = (inches / DriveConstants.WHEEL_CIRCUMFERENCE_INCHES) * DriveConstants.GEAR_RATIO * ConSparkMax.POSITION_CONVERSION_FACTOR;
    //System.out.println("tickstoinch" +encoderTicks);
    return encoderTicks;
  }

  public double getRightEncoderInches(){
    return m_rightEncoder.getPosition() * DriveConstants.INCHES_PER_TICK; 
  }
  public double getLeftEncoderInches(){
    return m_leftEncoder.getPosition() * DriveConstants.INCHES_PER_TICK; 
  }
  public double getAverageEncoderInches(){
    return (getRightEncoderInches() + getLeftEncoderInches())/2;
  }

  public double ticksToInches(double ticks) {
    //Converts Inches into Encoder ticks
    double inches = ticks / ConSparkMax.POSITION_CONVERSION_FACTOR / DriveConstants.GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE_INCHES;
    return inches;
  }

  public double getAverageEncoder(){
    return m_rightEncoder.getPosition() + m_leftEncoder.getPosition() / 2;
  }

  public static void ResetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

}
