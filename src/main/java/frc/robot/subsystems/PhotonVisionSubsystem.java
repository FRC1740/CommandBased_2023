// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVision. */
  //PhotonCamera m_rearCam = new PhotonCamera("rearCam");
  PhotonCamera m_frontCam = new PhotonCamera("OV9281");
  Transform3d robotToRearCam = new Transform3d(
    new Translation3d(Units.inchesToMeters(16), 0, 0), 
    new Rotation3d(0,0,0));
  Transform3d robotToFrontCam = new Transform3d(new Translation3d(Units.inchesToMeters(14), Units.inchesToMeters(4), Units.inchesToMeters(10)), new Rotation3d()); //Guess values for now

  AprilTagFieldLayout m_aprilTagFieldLayout;
  PhotonPoseEstimator m_photonPoseEstimator;

  public PhotonVisionSubsystem() {
    try{
    m_aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();}
    catch(IOException IOE){
      IOE.printStackTrace();
    }
    m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, m_frontCam, robotToFrontCam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Optional<EstimatedRobotPose> getEstimatedVisionPose() {
  return m_photonPoseEstimator.update();
}

  public double getXdeviationAprilTag(){

    if(m_frontCam.getLatestResult().hasTargets() == false){
      return 0;
    }else{
    return m_frontCam.getLatestResult().getBestTarget().getYaw();
    }
  }

  public double getDistanceToPose(Pose2d pose){
    return PhotonUtils.getDistanceToPose(getEstimatedVisionPose().get().estimatedPose.toPose2d(), pose);
  }

  public Rotation2d getYawToPose(Pose2d pose){
    return PhotonUtils.getYawToPose(getEstimatedVisionPose().get().estimatedPose.toPose2d(), pose);
  }

  public double getYawToPoseDegrees(Pose2d pose){
    return getYawToPose(pose).getDegrees();
  }
  // public double getYawSpecificAprilTag(int ID){
  //   double IDs[] = new double[0];
  //   List<PhotonTrackedTarget> tags = m_camera.getLatestResult().getTargets();
  //   for(PhotonTrackedTarget tag : tags){

  //   }

  // }

  public double getDistanceFromTag(){
    return m_frontCam.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
  }
}