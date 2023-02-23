// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.constants;

/** Add your docs here. */
public class OIConstants {
    public enum GamePiece {
        CUBE,
        CONE,
    };
    public static final GamePiece kDefaultGamePiece = GamePiece.CUBE;
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
}
