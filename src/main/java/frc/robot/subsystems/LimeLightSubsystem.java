// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.network.LimeLightTable;

public class LimeLightSubsystem extends SubsystemBase {

  private LimeLightTable m_LimeLightTable;

  public LimeLightSubsystem() {
    m_LimeLightTable = LimeLightTable.getInstance();
  }

  public double getXdeviation(){
    return m_LimeLightTable.getTx();
  }
  public double getYdeviation(){
    return m_LimeLightTable.getTy();
  }

  public void enableVisionProcessing(){
    m_LimeLightTable.setCamMode(0);
    m_LimeLightTable.setLedMode(3);
    System.out.println("Vision processing enabled");
  }
  public void enableDriverCamera(){
    m_LimeLightTable.setCamMode(1);
  }

  public double getCamMode(){
    return m_LimeLightTable.getCamMode();
  }
  //Toggle led on and off
  public void toggleLED(){
    if (m_LimeLightTable.getLedMode() == 1){
      m_LimeLightTable.setLedMode(3);
    }else{
      m_LimeLightTable.setLedMode(1);
    }
  }

  //Returns true if Limelight is in vision processing mode
  public BooleanSupplier isVisionProcessing(){
    BooleanSupplier camMode = () -> m_LimeLightTable.getCamMode() == 0;
    return camMode;
    
  }
  public void targetMidNode(){
    m_LimeLightTable.setPipeline(0);
  }

  public void targetHighNode(){
    m_LimeLightTable.setPipeline(1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}