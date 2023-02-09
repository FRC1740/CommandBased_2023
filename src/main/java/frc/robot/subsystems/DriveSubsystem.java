// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import frc.constants.DriveConstants;
import frc.constants.ShuffleboardConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
// import edu.wpi.first.math.filter.LinearFilter;

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

    private final DifferentialDriveOdometry m_odometry;
    private final DifferentialDrivePoseEstimator m_PoseEstimator;

    String circlePathTrajectoryJSON = "output/circlePath.wpilib.json";
    Trajectory circlePath = new Trajectory();
    String straightishTrajectoryJSON = "output/straightish.wpilib.json";
    Trajectory straightish = new Trajectory();
    Field2d m_Field = new Field2d();

    PhotonVision m_PhotonVision = new PhotonVision();
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
    // LinearFilter speedFilter;
    // LinearFilter rotationFilter;

    // Create widget for non-linear input
    GenericEntry m_nte_InputExponent;
    
    public DriveSubsystem() {
      m_leftMotorFollower.follow(m_leftMotorLeader);
      m_rightMotorFollower.follow(m_rightMotorLeader);  

      m_leftEncoder.setPositionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
      m_leftEncoderFollower.setPositionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
      m_rightEncoder.setPositionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
      m_rightEncoderFollower.setPositionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION_FACTOR);

      m_leftEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_FACTOR);
      m_leftEncoderFollower.setVelocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_FACTOR);
      m_rightEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_FACTOR);
      m_rightEncoderFollower.setVelocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_FACTOR);

      m_leftMotorLeader.burnFlash();
      m_leftMotorFollower.burnFlash();
      m_rightMotorLeader.burnFlash();
      m_rightMotorFollower.burnFlash();
      
      m_gyro.reset();
      m_gyro.calibrate();
      resetEncoders();

      m_odometry = new DifferentialDriveOdometry(getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
      
      //Pose estimator combines vision pose with odometry
      m_PoseEstimator = new DifferentialDrivePoseEstimator(
        DriveConstants.kDriveKinematics, 
        getRotation2d(), 
        0.0, 
        0.0, 
        new Pose2d());
      inst = NetworkTableInstance.getDefault();
      m_nt = inst.getTable(ShuffleboardConstants.DriveTrainTab);
      // Create and get reference to SB tab
      m_sbt_DriveTrain = Shuffleboard.getTab(ShuffleboardConstants.DriveTrainTab);
      
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
      m_nte_InputExponent = m_sbt_DriveTrain.addPersistent("Input Exponent", 1.0)
              .withSize(1, 1).withPosition(0, 2).getEntry();
  
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

      m_sbt_DriveTrain.add(m_Field);
      
      // speedFilter = LinearFilter.movingAverage(11);
      // rotationFilter = LinearFilter.movingAverage(5);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  
  public void arcadeDrive(double fwd, double rot, boolean squaredInput) {
    // double f_fwd = speedFilter.calculate(fwd);
    // double f_rot = rotationFilter.calculate(rot);
    m_drive.arcadeDrive(fwd, rot, squaredInput);
  }

  /**
  *    * Controls the left and right sides of the drive directly with voltages.
  *
  * @param leftVolts the commanded left output
  * @param rightVolts the commanded right output
  */
  public void tankDriveVolts(double leftVolts, double rightVolts){
    m_leftMotorLeader.setVoltage(leftVolts);
    m_rightMotorLeader.setVoltage(rightVolts);
    m_drive.feed();
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
    m_odometry.update(getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

    updatePoseEstimater();

    m_nte_LeftEncoder.setDouble(getAverageLeftEncoders());
    m_nte_RightEncoder.setDouble(getAverageRightEncoders());
    m_nte_IMU_ZAngle.setDouble(getAngle());
    m_nte_IMU_PitchAngle.setDouble(getRoll());
    }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    resetGyro();
    resetEncoders();
    m_odometry.resetPosition(getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters(), pose);
  }

  public void updatePoseEstimater(){
    m_PoseEstimator.update(getRotation2d(), getLeftDistanceInches(), getRightDistanceInches());
    
    Optional<EstimatedRobotPose> result = m_PhotonVision.getEstimatedVisionPose();

    if (result.isPresent()){
      EstimatedRobotPose visionPose = result.get();
      m_PoseEstimator.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds);
    }
    m_Field.setRobotPose(m_PoseEstimator.getEstimatedPosition());
  }

  public double getAngle() {
    //System.out.println("gyro angle" + m_gyro.getAngle());
    return m_gyro.getAngle();
  }
  
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public DifferentialDriveKinematics getDriveKinematics(){
    return DriveConstants.kDriveKinematics;
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
    return m_gyro.getRotation2d().getDegrees();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }
  public double getRightEncoderMeters(){
    return m_rightEncoder.getPosition(); 
  }
  public double getLeftEncoderMeters(){
    return m_leftEncoder.getPosition(); 
  }
  public double getAverageEncoderMeters(){
    return (getRightEncoderMeters() + getLeftEncoderMeters())/2;
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
    return Units.metersToInches(getAverageRightEncoders());
  }

  public double getLeftDistanceInches() {
    return Units.metersToInches(getAverageLeftEncoders());
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

  public double getAverageEncoder(){
    return m_rightEncoder.getPosition() + m_leftEncoder.getPosition() / 2;
  }

  public void ResetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public Command FollowPath(PathPlannerTrajectory trajectory, boolean isFirstPath) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        //Reset odometry for the first path ran during auto
        if(isFirstPath){
          this.resetOdometry(trajectory.getInitialPose());
        }
      }),
      new PPRamseteCommand(
        trajectory, 
        this::getPose,
        new RamseteController(),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kDriveKinematics,
        this::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        this::tankDriveVolts,
        false,
        this)
    );
  }

  public Trajectory getTrajectory(String TrajectoryJSON){
    
    try {
      Path TrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(TrajectoryJSON);
      return TrajectoryUtil.fromPathweaverJson(TrajectoryPath);

   }catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + circlePathTrajectoryJSON, ex.getStackTrace());
      return null;
   }

  }

  public Command getPathWeaverCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            getTrajectory(straightishTrajectoryJSON),
            this::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(
                DriveConstants.ks,
                DriveConstants.kv,
                DriveConstants.ka),
             DriveConstants.kDriveKinematics,
            this::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            this::tankDriveVolts,
            this);

    // Reset odometry to the starting pose of the trajectory.
    this.resetOdometry(getTrajectory(straightishTrajectoryJSON).getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
  }
}