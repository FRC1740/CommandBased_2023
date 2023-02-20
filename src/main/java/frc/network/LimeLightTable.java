// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightTable {

  private static String TableName = "limelight";

  private static String Tx_Entry = "tx";
  private static Double Tx_Default = 0.0;
  
  private static String Ty_Entry = "ty";
  private static Double Ty_Default = 0.0
  ;
  private static String Ta_Entry = "ta";
  private static Double Ta_Default = 0.0;

  private static String CamMode_Entry = "camMode";
  private static Double CamMode_Default = 0.0;

  private static String LedMode_Entry = "ledMode";
  private static Double LedMode_Default = 0.0;

  private static String Pipeline_Entry = "pipeline";
  private static Double Pipeline_Default = 0.0;

  NetworkTable m_nt = null;

  private static LimeLightTable instance = null;

  private LimeLightTable() {
    initNetworkTableInstance();
  }

  public static LimeLightTable getInstance() {
    if(instance == null) {
      instance = new LimeLightTable();
    }
    return instance;
  }

  private void initNetworkTableInstance() {
    m_nt = NetworkTableInstance.getDefault().getTable(TableName);
  }

  public double getTx() {
    return m_nt.getEntry(Tx_Entry).getDouble(Tx_Default);
  }
  
  public void setTx(double value) {
    m_nt.getEntry(Tx_Entry).setNumber(value);
  }
  
  public double getTy() {
    return m_nt.getEntry(Ty_Entry).getDouble(Ty_Default);
  }
  
  public void setTy(double value) {
    m_nt.getEntry(Ty_Entry).setNumber(value);
  }
  
  public double getTa() {
    return m_nt.getEntry(Ta_Entry).getDouble(Ta_Default);
  }
  
  public void setTa(double value) {
    m_nt.getEntry(Ta_Entry).setNumber(value);
  }
  
  public double getCamMode() {
    return m_nt.getEntry(CamMode_Entry).getDouble(CamMode_Default);
  }
  
  public void setCamMode(double value) {
    m_nt.getEntry(CamMode_Entry).setNumber(value);
  }
  
  public double getLedMode() {
    return m_nt.getEntry(LedMode_Entry).getDouble(LedMode_Default);
  }
  
  public void setLedMode(double value) {
    m_nt.getEntry(LedMode_Entry).setNumber(value);
  }
  
  public double getPipeline() {
    return m_nt.getEntry(Pipeline_Entry).getDouble(Pipeline_Default);
  }
  
  public void setPipeline(double value) {
    m_nt.getEntry(Pipeline_Entry).setNumber(value);
  }

}