// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Paths;
import frc.robot.RobotShared;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;


import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import frc.board.DriveTrainTab;
import frc.constants.AutoConstants;
import frc.constants.DriveConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
// import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;

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

    // LinearFilter speedFilter;
    LinearFilter rotationFilter;
    SlewRateLimiter speedLimiter;

    private DriveTrainTab m_DriveTrainTab;

    private RobotShared m_robotShared;
    private PhotonVisionSubsystem m_PhotonVision;
    
    public DriveSubsystem() {
      m_rightMotorLeader.setInverted(false);
      m_leftMotorLeader.setInverted(true);

      m_leftMotorFollower.follow(m_leftMotorLeader);
      m_rightMotorFollower.follow(m_rightMotorLeader);
      
      m_rightMotorLeader.setSmartCurrentLimit(50);
      m_rightMotorFollower.setSmartCurrentLimit(50);
      m_leftMotorLeader.setSmartCurrentLimit(50);
      m_leftMotorFollower.setSmartCurrentLimit(50);


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
        getLeftEncoderMeters(), 
        getRightEncoderMeters(), 
        new Pose2d());

      m_DriveTrainTab = DriveTrainTab.getInstance();
      
      // speedFilter = LinearFilter.movingAverage(10);
      rotationFilter = LinearFilter.movingAverage(3);
      speedLimiter = new SlewRateLimiter(DriveConstants.kDrivePositiveRateLimit,
          DriveConstants.kDriveNegativeRateLimit, 0);

      m_robotShared = RobotShared.getInstance();
      m_PhotonVision = m_robotShared.getPhotonVisionSubsystem();
  }

  // Helper for calculations below
  private double mapRange(double value, double low1, double high1, double low2, double high2) {
    double x = Math.max(low1, Math.min(value, high1));
    return low2 + (high2 - low2) * (x - low1) / (high1 - low1);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  
  public void arcadeDrive(double fwd, double rot, boolean squaredInput) {
    // Implement Kyle's option 2 for rotation modification
    // Motor voltages are the best proxy we have for velocity [-1, 1]
    double velocity = (m_leftMotorLeader.get() + m_rightMotorLeader.get()) / 2.0;
    double rotDeadzone = Math.abs(rot) < DriveConstants.kRotationDeadzone? 0.0 : 1.0;
    double rotBoost = mapRange(Math.abs(velocity),
      DriveConstants.kRotationVelocityLow, DriveConstants.kRotationVelocityHigh,
      DriveConstants.kRotationBoostLow, DriveConstants.kRotationBoostHigh);
    
    // double f_fwd = speedFilter.calculate(fwd);
    // Ignore filtering of input at low speeds
    // if (Math.abs(velocity) < DriveConstants.kMinVelocityForFilter) {
    //   f_fwd = fwd;
    // }
    double f_fwd = speedLimiter.calculate(fwd);
    double f_rot = rotationFilter.calculate(rot * rotBoost * rotDeadzone);
    m_drive.arcadeDrive(f_fwd, f_rot, squaredInput);
  }

  public void simpleArcadeDrive(double fwd, double rot, boolean squaredInput) {
    m_drive.arcadeDrive(fwd, rot, squaredInput);
  }
  /**
  *    * Controls the left and right sides of the drive directly with voltages.
  *
  * @param leftVolts the commanded left output
  * @param rightVolts the commanded right output
  */
  public void tankDriveVolts(double rightVolts, double leftVolts){ //right has to to on the left and left volts on the right(second parameter)
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
    m_DriveTrainTab.setIMU_PitchAngle(getPitch());
    m_DriveTrainTab.setRobotPose(getPose());
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public Pose2d getEstimatedVisionPose(){
    return m_PoseEstimator.getEstimatedPosition();
  }

  public void resetPoseEstimation(Pose2d pose){
    m_PoseEstimator.resetPosition(getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters(), pose);
  }

  public void resetOdometry(Pose2d pose){
    resetGyro();
    m_odometry.resetPosition(getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters(), pose);
  }

  public void updatePoseEstimater(){

    m_PoseEstimator.update(getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

    
    Optional<EstimatedRobotPose> result = m_PhotonVision.getEstimatedVisionPose();

    if (result.isPresent()){
      EstimatedRobotPose visionPose = result.get();
      m_PoseEstimator.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds);
    }
    //m_DriveTrainTab.setRobotPose(m_PoseEstimator.getEstimatedPosition());
  }

  public double getAngle() {
    //System.out.println("gyro angle" + m_gyro.getAngle());
    return m_gyro.getAngle();
  }

  public double getVisionAngle(){
    return getEstimatedVisionPose().getRotation().getDegrees();
  }
  
  public Rotation2d getRotation2d(){
    return m_gyro.getRotation2d();
  }

  public DifferentialDriveKinematics getDriveKinematics(){
    return DriveConstants.kDriveKinematics;
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public double getPitch(){
    return m_gyro.getPitch();
  }

  public double getRawGyroX(){
    
    return m_gyro.getRawGyroX();
  }

  public double getRoll(){
    return m_gyro.getRoll();
  }
  
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
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

  public double getAverageLeftEncoders() {
    return (m_leftEncoder.getPosition() + m_leftEncoderFollower.getPosition() ) / 2.0;
  }

  public double getAverageRightEncoders() {
    return (m_rightEncoder.getPosition() + m_rightEncoderFollower.getPosition() ) / 2.0;
  }
  
  public Command FollowPath(PathPlannerTrajectory trajectory, boolean isFirstPath) { // FIXME: COMMANDS SHOULD NOT BE INSTANTIATED INSIDE A SUBSYSTEM!!!
    m_DriveTrainTab.setTrajectory(PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance()));
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        //Reset odometry for the first path ran during auto
        if(isFirstPath){
          this.resetPoseEstimation(PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance()).getInitialPose());
          this.resetOdometry(PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance()).getInitialPose());
        }
      }),
      new PPRamseteCommand(
        trajectory, 
        this::getPose,
        new RamseteController(),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kDriveKinematics,
        this::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel*0, 0, 0),
        new PIDController(DriveConstants.kPDriveVel*0, 0, 0),
        this::tankDriveVolts,
        true,
        this)
    );
  }

  public Command FollowPathWithEvents(PathPlannerTrajectory path, boolean isFirstPath) {
    return new FollowPathWithEvents(FollowPath(path, isFirstPath), path.getMarkers(), Paths.getEventMap());
  }

  public Command FollowPathVision(PathPlannerTrajectory trajectory, boolean isFirstPath) { // FIXME: COMMANDS SHOULD NOT BE INSTANTIATED INSIDE A SUBSYSTEM!!!
    m_DriveTrainTab.setTrajectory(PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance()));
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        //Reset odometry for the first path ran during auto
        if(isFirstPath){
          this.resetPoseEstimation(PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance()).getInitialPose());
          this.resetOdometry(PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance()).getInitialPose());
        }
      }),
      new PPRamseteCommand(
        trajectory, 
        this::getEstimatedVisionPose,
        new RamseteController(),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kDriveKinematics,
        this::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel*0, 0, 0),
        new PIDController(DriveConstants.kPDriveVel*0, 0, 0),
        this::tankDriveVolts,
        true,
        this)
    );
  }
//   public Trajectory getTrajectory(String TrajectoryJSON){
    
//     try {
//       Path TrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(TrajectoryJSON);
//       return TrajectoryUtil.fromPathweaverJson(TrajectoryPath);

//    }catch (IOException ex) {
//       DriverStation.reportError("Unable to open trajectory: " + circlePathTrajectoryJSON, ex.getStackTrace());
//       return null;
//    }

//   }

//   public Command getPathWeaverCommand(boolean isFirstPath) {

// Trajectory trajectory = getTrajectory(straightishTrajectoryJSON);
//     m_DriveTrainTab.setTrajectory(trajectory);
//     RamseteCommand ramseteCommand =
//         new RamseteCommand(
//             trajectory,
//             this::getEstimatedVisionPose,
//             new RamseteController(),
//             new SimpleMotorFeedforward(
//                 DriveConstants.ks,
//                 DriveConstants.kv,
//                 DriveConstants.ka),
//              DriveConstants.kDriveKinematics,
//             this::getWheelSpeeds,
//             new PIDController(DriveConstants.kPDriveVel, 0, 0),
//             new PIDController(DriveConstants.kPDriveVel, 0, 0),
//             // RamseteCommand passes volts to the callback
//             this::tankDriveVolts,
//             this);

//     // Run path following command, then stop at the end.
//     return new SequentialCommandGroup(
//       new InstantCommand(() -> {
//         //Reset odometry for the first path ran during auto
//         if(isFirstPath){
//           this.resetPoseEstimation(trajectory.getInitialPose());
//           this.resetOdometry(trajectory.getInitialPose());
//         }
//       }
//     ),
//     ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0)));
//   }

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

  public Pose2d getAdjustedPose(Pose2d pose){
    if (DriverStation.getAlliance() == Alliance.Red){
      return new Pose2d(AutoConstants.kFieldLengthMeters-pose.getX(),AutoConstants.kFieldWidthMeters - pose.getY(), pose.getRotation());
    } else {
      return pose;
    }
  }
}