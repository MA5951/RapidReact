// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

/** Add your docs here. */
public class ClimbConstants {
    public static final String SYSTEM_NAME = "Shooter";

    public static final double ROTATION_KP = 0.6;
    public static final double ROTATION_KI = 0;
    public static final double ROTATION_KD = 0.045;
    public static final double ROTATION_TOLERANCE = 0.2;

    public static final double EXTENSION_KP = 0.0035;
    public static final double EXTENSION_KI = 0.00002;
    public static final double EXTENSION_KD = 0;
    public static final double EXTENSION_TOLERANCE = 500;

    public static final double PASSIVE_KP = 0;
    public static final double PASSIVE_KI = 0;
    public static final double PASSIVE_KD = 0;
    public static final double PASSIVE_TOLERANCE = 0;

    public static final double MAX_POSITION = -88775;
}
