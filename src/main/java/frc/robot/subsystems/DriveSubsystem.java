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
import frc.constants.DriveConstants;
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.*;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new Drivetrain. */
    private final CANSparkMax m_leftMotorLeader = new CANSparkMax(DriveConstants.kLeftMotor1Port, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax m_rightMotorLeader = new CANSparkMax(DriveConstants.kRightMotor1Port, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_leftMotorFollower = new CANSparkMax(DriveConstants.kLeftMotor2Port, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_rightMotorFollower = new CANSparkMax(DriveConstants.kRightMotor2Port, CANSparkMaxLowLevel.MotorType.kBrushless);
    public final RelativeEncoder m_leftEncoder = m_leftMotorLeader.getEncoder();
    public final RelativeEncoder m_leftEncoderFollower = m_leftMotorFollower.getEncoder();
    public final RelativeEncoder m_rightEncoder = m_rightMotorLeader.getEncoder();
    public final RelativeEncoder m_rightEncoderFollower = m_rightMotorFollower.getEncoder();
    // gyro NavX IMU CRASHIN
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotorLeader, m_rightMotorLeader);

    // Used to grab an instance of the global network tables
    NetworkTableInstance inst;
    NetworkTable m_nt;
    Shuffleboard m_sb;

    // Shuffleboard DriveTrain entries
    // Create and get reference to SB tab
    ShuffleboardTab m_sbt_DriveTrain;
  
    GenericEntry m_nte_Testing;
  
    // Autonomous Variables
    GenericEntry m_nte_a_DriveDelay;
    GenericEntry m_nte_b_DriveDistance;
    GenericEntry m_nte_c_DriveTurnAngle;
    GenericEntry m_nte_autoDriveMode;

    // Encoders/PID Feedback sensors
    GenericEntry m_nte_LeftEncoder;
    GenericEntry m_nte_RightEncoder;
    GenericEntry m_nte_IMU_ZAngle;
    GenericEntry m_nte_IMU_PitchAngle; // USES IMU ROLL AXIS!!!

    // PID Tuning
    GenericEntry m_nte_DriveSpeedFilter;
    GenericEntry m_nte_DriveRotationFilter;

    // Create widget for non-linear input
    GenericEntry m_nte_InputExponent;
    
    public DriveSubsystem() {
      m_leftMotorFollower.follow(m_leftMotorLeader);
      m_rightMotorFollower.follow(m_rightMotorLeader);  

      inst = NetworkTableInstance.getDefault();
      m_nt = inst.getTable("drivetrain");
      // Create and get reference to SB tab
      m_sbt_DriveTrain = Shuffleboard.getTab("DriveTrain");
      
      // get a topic from a NetworkTableInstance
      // the topic name in this case is the full name
      //DoubleTopic dblTopic = inst.getDoubleTopic("/drivetrain/gyro");
      
      // get a topic from a NetworkTable
      // the topic name in this case is the name within the table;
      // this line and the one above reference the same topic
      // DoubleTopic dtGyro = m_nt.getDoubleTopic("gyro");
      
      // get a type-specific topic from a generic Topic
      // Topic genericTopic = inst.getTopic("/datatable/X");
      // DoubleTopic dblTopic = new DoubleTopic(genericTopic);
  
      // Create widgets for digital filter lengths
      m_nte_DriveSpeedFilter = m_sbt_DriveTrain.addPersistent("Drive Speed Filter", 10.0)
            .withSize(2, 1).withPosition(0, 0).getEntry();
  
      m_nte_DriveRotationFilter = m_sbt_DriveTrain.addPersistent("Drive Rotation Filter", 5.0)
            .withSize(2, 1).withPosition(0, 1).getEntry();
  
      // Create widget for non-linear input
      m_nte_InputExponent = m_sbt_DriveTrain.addPersistent("Input Exponent", 1.0)        .withSize(1, 1).withPosition(0, 2).getEntry();
  
      // Create widgets for AutoDrive
      m_nte_a_DriveDelay     = m_sbt_DriveTrain.addPersistent("Drive Delay", .5)
            .withSize(1, 1).withPosition(3, 0).getEntry();
      m_nte_b_DriveDistance  = m_sbt_DriveTrain.addPersistent("Drive Distance", 48)
            .withSize(1, 1).withPosition(3, 1).getEntry();
      m_nte_c_DriveTurnAngle = m_sbt_DriveTrain.addPersistent("Turn Angle", 0.0)
            .withSize(1, 1).withPosition(3, 2).getEntry();
      m_nte_autoDriveMode    = m_sbt_DriveTrain.addPersistent("AutoDrive Mode", 2)
            .withSize(1, 1).withPosition(3, 3).getEntry();

      //  m_nte_Testing     = m_sbt_DriveTrain.addPersistent("Testing", 0.0)       .withSize(1, 1).withPosition(3, 3).getEntry();
  
      // Encoder outputs
      // Display current encoder values
      m_nte_LeftEncoder = m_sbt_DriveTrain.addPersistent("Left Side Encoder", 0.0)
                  .withSize(2,1).withPosition(4,0).getEntry();
  
      m_nte_RightEncoder = m_sbt_DriveTrain.addPersistent("Right Side Encoder", 0.0)
                .withSize(2,1).withPosition(4,1).getEntry();
  
      m_nte_IMU_ZAngle = m_sbt_DriveTrain.addPersistent("IMU Z-Axis Angle", 0.0)
                .withSize(2,1).withPosition(4,2).getEntry();
  
      m_nte_IMU_PitchAngle = m_sbt_DriveTrain.addPersistent("IMU Pitch", 0.0)
                .withSize(2,1).withPosition(4,3).getEntry();

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
    m_nte_LeftEncoder.setDouble(getAverageLeftEncoders());
    m_nte_RightEncoder.setDouble(getAverageRightEncoders());
    m_nte_IMU_ZAngle.setDouble(getAngle());
    m_nte_IMU_PitchAngle.setDouble(getRoll());
    }

  public double getAngle() {
    //System.out.println("gyro angle" + m_gyro.getAngle());
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

    // Used by AutoDriveDistance
  public void resetEncoders() {
    m_rightEncoder.setPosition(0.0);
    m_leftEncoder.setPosition(0.0);
    m_rightEncoderFollower.setPosition(0.0);
    m_leftEncoderFollower.setPosition(0.0);
  }

  // Account for two encoders per side
  public double getRightDistanceInches() {
    return (getAverageRightEncoders() * DriveConstants.INCHES_PER_TICK);
  }

  public double getLeftDistanceInches() {
    return (getAverageLeftEncoders() * DriveConstants.INCHES_PER_TICK);
  }

  // Used by AutoDriveDistance
  public double getAverageDistanceInches() {
    return ((getLeftDistanceInches() + getRightDistanceInches()) / 2.0);
  }

  public double getAverageLeftEncoders() {
    return (m_leftEncoder.getPosition() + m_leftEncoderFollower.getPosition() ) / 2.0;
  }

  public double getAverageRightEncoders() {
    return (m_rightEncoder.getPosition() + m_rightEncoderFollower.getPosition() ) / 2.0;
  }

  public double ticksToInches(double ticks) {
    //Converts Inches into Encoder ticks
    double inches = ticks / ConSparkMax.POSITION_CONVERSION_FACTOR / DriveConstants.GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE_INCHES;
    return inches;
  }

  public double getAverageEncoder(){
    return m_rightEncoder.getPosition() + m_leftEncoder.getPosition() / 2;
  }

  public void ResetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

}
