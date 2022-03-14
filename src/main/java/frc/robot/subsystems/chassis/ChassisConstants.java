// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import edu.wpi.first.math.system.plant.DCMotor;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.Calculations;

/**
 * Add your docs here.
 */
public class ChassisConstants {
    public static final String KSUBSYSTEM_NAME = "Chassis";

    public static final double MAX_VELOCITY = 8;
    public static final double MAX_ACCELERATION = 8;
    public static final double TRACK_WIDTH = 0.70; // cm
    public static final double MAX_VELOCITY_MPS= 3.5;

    // ------------------right pid velocity-------------
    public static final double KP_MAPATH_RIGHT_VELOCITY = 0.3;
    public static final double KI_MAPATH_RIGHT_VELOCITY = 0;
    public static final double KD_MAPATH_RIGHT_VELOCITY = 0;
    public static final double KF_MAPATH_RIGHT_VELOCITY = 1;
    public static final double KV_MAPATH_RIGHT_VELOCITY = 0.06;

    // ------------------left pid velocity-------------
    public static final double KP_MAPATH_LEFT_VELOCITY = 0.3;
    public static final double KI_MAPATH_LEFT_VELOCITY = 0;
    public static final double KD_MAPATH_LEFT_VELOCITY = 0;
    public static final double KF_MAPATH_LEFT_VELOCITY = 1;
    public static final double KV_MAPATH_LEFT_VELOCITY = 0.06;

    public static final double KP_MAPATH_ANGLE = 0.3;
    public static final double KI_MAPATH_ANGLE = 0;
    public static final double KD_MAPATH_ANGLE = 0.02;

    public static final double KP_VISION_ANGLE = 0.42;
    public static final double KI_VISION_ANGLE = 0;
    public static final double KD_VISION_ANGLE = 0.05;

    public static final double KP_VISION_DISTANCE = 1.6e-2;
    public static final double KI_VISION_DISTANCE = 0;
    public static final double KD_VISION_DISTANCE = 0;

    public static final double KANGLE_PID_VISION_SET_INPUTRANGE = 44.5;
    public static final double KANGLE_PID_MAPATH_SET_INPUTRANGE = 180;

    public static final double KRAMP_RATE_AUTO = 0.35;
    public static final double KTHRESHOLD = 0.1;
    public static final double KSCALE = 0.3;

    public static final double KchassisLength = 0.83; // TODO
    public static final double KwhellRadius = 0.508;

    public static final double KSUB_MAS = 26;
    public static final int KTICKS_PER_METER = 54150;
    public final static double KMOTOR_GEAR_RATIO = 1.0 / 9;
    public static final int KTICKS_PER_REVOLUTION = 2048;
    public static final double KMETER_PER_TICKS = (2 * Math.PI * KwhellRadius)
            / (KTICKS_PER_REVOLUTION / KMOTOR_GEAR_RATIO);
    public final static double KSPROCKET_RADIUS = 0.016;// TOOD
    public static final double KCHASSIS_GEAR = KwhellRadius * KMOTOR_GEAR_RATIO; // TODO
    public final static double KMOTOR_FORCE = ((1 / KMOTOR_GEAR_RATIO) * RobotConstants.KSTALL_TORQUE_FALCON * 2)
            / KwhellRadius;
    public static final double KMAX_SPEED = Calculations.fromRPMToLinearSpeed(DCMotor.getNEO(2).freeSpeedRadPerSec * 10,
            KCHASSIS_GEAR); // TODO
    public static final double KMAX_ACCELERATION = KMOTOR_FORCE * 2 / KSUB_MAS;
}