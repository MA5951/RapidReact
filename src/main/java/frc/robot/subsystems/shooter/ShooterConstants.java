package frc.robot.subsystems.shooter;

public class ShooterConstants {
    public static final String SYSTEM_NAME = "Shooter";

    public static final double SHOOTER_VELOCITY_PID_KP = 0.0005;
    public static final double SHOOTER_VELOCITY_PID_KI = 0.00007;
    public static final double SHOOTER_VELOCITY_PID_KD = 0;
    public static final double SHOOTER_VELOCITY_PID_KF = 1.1;
    public static final double SHOOTER_VELOCITY_PID_TOLERANCE = 50;

    public static final int SHOOTER_VELOCITY_LAUNCH_PAD = -2500;//3200;
    public static final int SHOOTER_VELOCITY_FENDER = 2500;
}
