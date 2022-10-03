package frc.robot.subsystems.shooter;

public class ShooterConstants {
    public static final String SYSTEM_NAME = "Shooter";

    public static final double SHOOTER_VELOCITY_PID_KP = 0.002;
    public static final double SHOOTER_VELOCITY_PID_KI = 0.00007;
    public static final double SHOOTER_VELOCITY_PID_KD = 0;
    public static final double SHOOTER_VELOCITY_PID_KF = 0.95;
    public static final double SHOOTER_VELOCITY_PID_TOLERANCE = 100;

    public static final double SHOOTER_VELOCITY_LAUNCH_PAD = -2550;
    public static final double SHOOTER_VELOCITY_FENDER = 2200; // -2200;

    public static final double SHOOTER_ANGLE = 43.64;
    public static final double K_DELTA_Y = 1.64; // 1.64
}
