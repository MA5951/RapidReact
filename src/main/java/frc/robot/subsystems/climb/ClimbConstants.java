// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

/** Add your docs here. */
public class ClimbConstants {
    public static final String SYSTEM_NAME = "Climb";

    public static final double ROTATION_KP = 0.001;
    public static final double ROTATION_KI = 0.000002; 
    public static final double ROTATION_KD = 0.0002;
    public static final double ROTATION_TOLERANCE = 100;

    public static final double EXTENSION_KP = 0.0005;
    public static final double EXTENSION_KI = 0;
    public static final double EXTENSION_KD = 0;
    public static final double EXTENSION_TOLERANCE =2500;

    public static final double PASSIVE_KP = 0;
    public static final double PASSIVE_KI = 0;
    public static final double PASSIVE_KD = 0;
    public static final double PASSIVE_TOLERANCE = 0;

    public static final double MAX_POSITION = 0.53;
    public static final double TICK_PER_METER_EXTENSION = -155900 * 2;
    public static final double TICK_FOR_90_DEGREES_ROTATION = -53000;
    

}
