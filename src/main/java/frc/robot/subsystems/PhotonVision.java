// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  private static PhotonCamera m_camera = new PhotonCamera("Webcam1");
  
  public PhotonVision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public double getXdeviationAprilTag(){
    if(m_camera.getLatestResult().hasTargets() == false){
      return 0;
    }else{
    return m_camera.getLatestResult().getBestTarget().getYaw();
    }
  }
}