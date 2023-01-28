// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */
  NetworkTable table;
  double tx;
  double ty;
  double ta;

  public LimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx").getDouble(0.0);
    ty = table.getEntry("ty").getDouble(0.0);
    ta = table.getEntry("ta").getDouble(0.0);

//read values periodically

  }
  public double getXdeviation(){
    return table.getEntry("tx").getDouble(0.0);
  }
  public double getYdeviation(){
    return table.getEntry("ty").getDouble(0.0);
  }

  public void enableVisionProcessing(){
    table.getEntry("camMode").setNumber(0);
    table.getEntry("ledMode").setNumber(3);
    System.out.println("Vision processing enabled");
  }
  public void enableDriverCamera(){
    table.getEntry("camMode").setNumber(1);
  }

  public double getCamMode(){
    return table.getEntry("camMode").getDouble(0);
  }
  //Toggle led on and off
  public void toggleLED(){
   if (table.getEntry("ledMode").getDouble(0) == 1){
    table.getEntry("ledMode").setNumber(3);
   }else{
    table.getEntry("ledMode").setNumber(1);
   }
  }

  //Returns true if Limelight is in vision processing mode
  public BooleanSupplier isVisionProcessing(){
    BooleanSupplier camMode = () -> table.getEntry("camMode").getDouble(1) == 0;
    return camMode;
    
  }
  public void targetMidNode(){
    table.getEntry("pipeline").setNumber(0);
  }

  public void targetHighNode(){
    table.getEntry("pipeline").setNumber(1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}