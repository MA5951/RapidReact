package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class PortMap {
    // Chassis
    public static final int chassisLeftFrontMotor = 2;
    public static final int chassisRightFrontMotor = 3;
    public static final int chassisLeftRearMotor = 4;
    public static final int chassisRightRearMotor = 5;

    // Intake
    public static final int intakeMotor = 6;
    public static final int intakePistonForward = 3;
    public static final int intakePistonReverse = 2;
    // Conveyor
    public static final int conveyorLowerMotor = 7;
    public static final int conveyorUpperMotor = 8;
    public static final int conveyorLowerIR = 0;
    public static final int conveyorUpperIR = 2;

    // Shooter
    public static final int shooterLeftMotor = 9;
    public static final int shooterRightMotor = 10;
    public static final int shooterPistonForward = 7;
    public static final int shooterPistonReverse = 8;

    // Climb
    public static final int climbExtensionMotor = 11;
    public static final int climbRotationLeftMotor = 12;
    public static final int climbRotationRightMotor = 13;
    public static final int climbPassiveMotor = 14;

    // LED
    public static final int leftLeds = 0;
    public static final int rightLeds = 1;
}
