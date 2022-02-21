// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.controllers.PIDController;
import com.ma5951.utils.motor.MA_SparkMax;
import com.ma5951.utils.subsystem.ControlSubsystem;

public class ClimbRotation extends SubsystemBase implements ControlSubsystem {
    /**
     * Climb Rotation Arm
     */
    private static ClimbRotation climbRotation;
    private MA_SparkMax leftRotationMotor, rightRotationMotor;
    private PIDController rotationPID;

    public ClimbRotation() {
        leftRotationMotor = new MA_SparkMax(PortMap.climbRotationLeftMotor, false, RobotConstants.KMOTOR_BRAKE,
                RobotConstants.LIMIT_SWITCH.forward, RobotConstants.ENCODER.No_Encoder,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        // leftRotationMotor.enableLimitSwitchF(true);

        rightRotationMotor = new MA_SparkMax(PortMap.climbRotationRightMotor, false, RobotConstants.KMOTOR_BRAKE,
                RobotConstants.ENCODER.No_Encoder, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightRotationMotor.follow(leftRotationMotor);

        rotationPID = new PIDController(ClimbConstants.ROTATION_KP, ClimbConstants.ROTATION_KI,
                ClimbConstants.ROTATION_KD,
                0, ClimbConstants.ROTATION_TOLERANCE, -12, 12);
    }

    public static ClimbRotation getInstance() {
        if (climbRotation == null) {
            climbRotation = new ClimbRotation();
        }
        return climbRotation;
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
        return rotationPID.calculate(leftRotationMotor.getPosition());
    }

    @Override
    public void setVoltage(double voltage) {
        leftRotationMotor.setVoltage(voltage);
    }

    public double getVoltage() {
        return leftRotationMotor.getOutput() * 12;
    }

    public boolean canMove(){
        return true; //TODO
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
