// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.chassis.Chassis;

import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.Shuffleboard;
import com.ma5951.utils.controllers.PIDController;
import com.ma5951.utils.motor.MA_SparkMax;
import com.ma5951.utils.subsystem.ControlSubsystem;

public class ClimbRotation extends SubsystemBase implements ControlSubsystem {
    /**
     * Climb Rotation Arm
     */
    private static ClimbRotation climbRotation;
    private MA_SparkMax leftRotationMotor, rightRotationMotor;
    private DigitalInput hallEffect;
    private PIDController rotationPID;
    private Shuffleboard shuffleboard;
    public double feedforward = 1;
    public double setPoint = 0;

    public ClimbRotation() {
        leftRotationMotor = new MA_SparkMax(PortMap.climbRotationLeftMotor, false, RobotConstants.KMOTOR_BRAKE,
                RobotConstants.ENCODER.Encoder, CANSparkMaxLowLevel.MotorType.kBrushless);

        rightRotationMotor = new MA_SparkMax(PortMap.climbRotationRightMotor, false, RobotConstants.KMOTOR_BRAKE,
                RobotConstants.ENCODER.Encoder, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightRotationMotor.follow(leftRotationMotor);

        hallEffect = new DigitalInput(1);

        rotationPID = new PIDController(ClimbConstants.ROTATION_KP, ClimbConstants.ROTATION_KI,
                ClimbConstants.ROTATION_KD,
                0, ClimbConstants.ROTATION_TOLERANCE, -12, 12);
        shuffleboard = new Shuffleboard("ClimbRotation");
        feedforward = 1;
    }

    public static ClimbRotation getInstance() {
        if (climbRotation == null) {
            climbRotation = new ClimbRotation();
        }
        return climbRotation;
    }

    public boolean getHallEffect() {
        return !hallEffect.get();
    }

    @Override
    public void setSetpoint(double setPoint) {
        rotationPID.setSetpoint(setPoint);
    }

    @Override
    public boolean atSetpoint() {
        return rotationPID.atSetpoint();
    }

    @Override
    public double calculate() {
        return rotationPID.calculate(leftRotationMotor.getPosition()) * feedforward;
    }

    @Override
    public void setVoltage(double voltage) {
        leftRotationMotor.setVoltage(voltage);
    }

    public double getVoltage() {
        return leftRotationMotor.getOutput() * 12;
    }

    public double getCurrent() {
        return (leftRotationMotor.getStatorCurrent() + rightRotationMotor.getStatorCurrent()) / 2.0;
    }

    public boolean canMove() {
        return getCurrent() > 20;
    }

    public void reset() {
        leftRotationMotor.resetEncoder();
        rightRotationMotor.resetEncoder();
    }

    @Override
    public void periodic() {

        if (getHallEffect()) {
            leftRotationMotor.resetEncoder();
            rightRotationMotor.resetEncoder();
        }
        shuffleboard.addBoolean("hallEffect", getHallEffect());
        shuffleboard.addBoolean("open passive?", Chassis.getinstance().canOpenPassiveArm());

        // This method will be called once per scheduler run
        shuffleboard.addNum("pid", calculate());
        shuffleboard.addNum("encoder left", leftRotationMotor.getPosition());
        shuffleboard.addNum("encoder right", rightRotationMotor.getPosition());
        shuffleboard.addNum("setPoint", rotationPID.getSetpoint());
        // feedforward = shuffleboard.getNum("F");
    }
}
