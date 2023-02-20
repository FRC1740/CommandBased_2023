// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVision. */
  PhotonCamera m_camera = new PhotonCamera("FrontCam");
  Transform3d robotToCam = new Transform3d(
    new Translation3d(Units.inchesToMeters(16), 0, 0), 
    new Rotation3d(0,0,0));

  AprilTagFieldLayout m_aprilTagFieldLayout;
  PhotonPoseEstimator m_photonPoseEstimator;

  public PhotonVisionSubsystem() {
    try{
    m_aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();}
    catch(IOException IOE){
      IOE.printStackTrace();
    }
    m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, m_camera, robotToCam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Optional<EstimatedRobotPose> getEstimatedVisionPose() {
  return m_photonPoseEstimator.update();
}

  public double getXdeviationAprilTag(){
    if(m_camera.getLatestResult().hasTargets() == false){
      return 0;
    }else{
    return m_camera.getLatestResult().getBestTarget().getYaw();
    }
  }
}