// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.chassis.Chassis;

import com.ma5951.utils.Shuffleboard;
import com.ma5951.utils.controllers.PIDController;
import com.ma5951.utils.subsystem.ControlSubsystem;

public class ClimbRotation extends SubsystemBase implements ControlSubsystem {
    /**
     * Climb Rotation Arm
     */
    private static ClimbRotation climbRotation;
    private TalonFX leftRotationMotor, rightRotationMotor;
    private DigitalInput hallEffect;
    private PIDController rotationPID;
    private Shuffleboard shuffleboard;
    public double feedforward = 1;
    public double setPoint = 0;
    public double shuffleBoardFeedforward = 1;

    public ClimbRotation() {
        leftRotationMotor = new TalonFX(PortMap.climbRotationLeftMotor);
        rightRotationMotor = new TalonFX(PortMap.climbRotationRightMotor);

        leftRotationMotor.setNeutralMode(NeutralMode.Brake);
        rightRotationMotor.setNeutralMode(NeutralMode.Brake);


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

    public double averageDis(){
        return (leftRotationMotor.getSelectedSensorPosition() + rightRotationMotor.getSelectedSensorPosition())/2;
    }

    public boolean getHallEffect() {
        return !hallEffect.get();
    }

    @Override
    public void setSetpoint(double setPoint) {
       rotationPID.setSetpoint((setPoint/90) * ClimbConstants.TICK_FOR_90_DEGREES_ROTATION);
    }

    @Override
    public boolean atSetpoint() {
        return rotationPID.atSetpoint();
    }

    @Override
    public double calculate() {
        return rotationPID.calculate(averageDis()) * feedforward;
    }

    @Override
    public void setVoltage(double voltage) {
        leftRotationMotor.set(TalonFXControlMode.PercentOutput, voltage / 12.0);//leftRotationMotor.setVoltage(voltage);
    }

    public double getVoltage() {
        return leftRotationMotor.getMotorOutputVoltage();
    }

    public double getCurrent() {
        return (leftRotationMotor.getStatorCurrent() + rightRotationMotor.getStatorCurrent()) / 2.0;
    }

    public boolean canMove() {
        return getCurrent() > 20;
    }

    public void reset() {
        leftRotationMotor.setSelectedSensorPosition(0);
        rightRotationMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        if (getHallEffect()) {
           reset();
        }
        shuffleboard.addBoolean("hallEffect", getHallEffect());
        shuffleboard.addBoolean("open passive?", Chassis.getinstance().canOpenPassiveArm());
        shuffleboard.addBoolean("At setpoint", rotationPID.atSetpoint());

        // This method will be called once per scheduler run
        shuffleboard.addNum("pid", calculate());
        shuffleboard.addNum("averageDis", (averageDis() * 90) / ClimbConstants.TICK_FOR_90_DEGREES_ROTATION);
        shuffleboard.addNum("setPoint", (rotationPID.getSetpoint() * 90) / ClimbConstants.TICK_FOR_90_DEGREES_ROTATION);

        shuffleBoardFeedforward = shuffleboard.getNum("Rotation Feedforward");

    
       
    }
}
