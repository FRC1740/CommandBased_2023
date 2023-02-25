// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.constants;

/** Add your docs here. */
public class GroundIntakeConstants {
    public static final int kIntakeMotorPort = 10; 
    public static final int kPneumaticPortA = 1;
    public static final int kPneumaticPortB = 0;

    public static final double kDefaultIntakeSpeed = -0.5;

    public static final double kConeIntakeSpeed  = -0.5;
    public static final double kConeEjectSpeed   = -kConeIntakeSpeed;
    public static final double kCubeIntakeSpeed  = -0.5;
    public static final double kCubeEjectSpeed   = -kCubeIntakeSpeed;
    public static final double kConeGraspSpeed   =  kConeIntakeSpeed;
    public static final double kCubeGraspSpeed   =  kCubeIntakeSpeed;

    public static final double kIntakeGearRatio = 10.71; // FIXME: Actual Ground Intake Gear Ratio?
}