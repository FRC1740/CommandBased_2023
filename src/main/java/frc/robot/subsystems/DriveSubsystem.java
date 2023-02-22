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

import frc.board.DriveTrainTab;
import frc.constants.DriveConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.RelativeEncoder;
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
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.LinearFilter;

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
    String StraightTrajectoryJSON = "output/Straight.wpilib.json";
    Trajectory Straight = new Trajectory();

    PhotonVisionSubsystem m_PhotonVision = new PhotonVisionSubsystem(); // FIXME: We should NOT be instantiating a new subsystem here! RobotContainer does that. use getInstance()!!

    LinearFilter speedFilter;
    LinearFilter rotationFilter;

    private DriveTrainTab m_DriveTrainTab;
    
    public DriveSubsystem() {
      m_rightMotorLeader.setInverted(false);
      m_leftMotorLeader.setInverted(true);

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

      m_DriveTrainTab = DriveTrainTab.getInstance();
      
      speedFilter = LinearFilter.movingAverage(30);
      rotationFilter = LinearFilter.movingAverage(5);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  
  public void arcadeDrive(double fwd, double rot, boolean squaredInput) {
    double f_fwd = speedFilter.calculate(fwd);
    double f_rot = rotationFilter.calculate(rot);
    m_drive.arcadeDrive(f_fwd, f_rot, squaredInput);
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

    m_DriveTrainTab.setLeftEncoder(getAverageLeftEncoders());
    m_DriveTrainTab.setRightEncoder(getAverageRightEncoders());
    m_DriveTrainTab.setIMU_ZAngle(getAngle());
    m_DriveTrainTab.setIMU_PitchAngle(getRoll());
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public Pose2d getEstimatedVisionPose(){
    return m_PoseEstimator.getEstimatedPosition();
  }

  public void resetPoseEstimation(Pose2d pose){
    resetEncoders();
    m_PoseEstimator.resetPosition(getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters(), pose);
  }

  public void resetOdometry(Pose2d pose){
    resetGyro();
    resetEncoders();
    m_odometry.resetPosition(getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters(), pose);
  }

  public void updatePoseEstimater(){

    m_PoseEstimator.update(getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

    
    Optional<EstimatedRobotPose> result = m_PhotonVision.getEstimatedVisionPose();

    if (result.isPresent()){
      EstimatedRobotPose visionPose = result.get();
      m_PoseEstimator.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds);
    }
    m_DriveTrainTab.setRobotPose(m_PoseEstimator.getEstimatedPosition());
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

  // public void ResetEncoders() {
  //   m_leftEncoder.setPosition(0);
  //   m_rightEncoder.setPosition(0);
  // }

  
  public Command FollowPath(PathPlannerTrajectory trajectory, boolean isFirstPath) { // FIXME: COMMANDS SHOULD NOT BE INSTANTIATED INSIDE A SUBSYSTEM!!!
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        //Reset odometry for the first path ran during auto
        if(isFirstPath){
          resetPoseEstimation(trajectory.getInitialPose());
        }
      }),
      new PPRamseteCommand(
        trajectory, 
        this::getEstimatedVisionPose,
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
            this::getEstimatedVisionPose,
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
    resetEncoders();
    resetGyro();
    resetPoseEstimation(getTrajectory(straightishTrajectoryJSON).getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
  }

// public Command getManualTrajectoryCommand(){
//   DifferentialDriveVoltageConstraint autoVoltageConstraint = 
//     new DifferentialDriveVoltageConstraint(
//       new SimpleMotorFeedforward(
//         DriveConstants.ks, 
//         DriveConstants.kv, 
//         DriveConstants.ka),
//       DriveConstants.kDriveKinematics,
//       10);

//   TrajectoryConfig config =
//     new TrajectoryConfig(
//       DriveConstants.kMaxSpeedMetersPerSecond,
//       DriveConstants.kMaxAccelerationMetersPerSecondSquared)
//       .setKinematics(DriveConstants.kDriveKinematics)
//       .addConstraint(autoVoltageConstraint);

//   Trajectory testTrajectory = 
//     TrajectoryGenerator.generateTrajectory(null, null, null, config)
//   }
  public void burnFlash() {
    m_leftMotorLeader.burnFlash();
    m_leftMotorFollower.burnFlash();
    m_rightMotorLeader.burnFlash();
    m_rightMotorFollower.burnFlash();
  }
}