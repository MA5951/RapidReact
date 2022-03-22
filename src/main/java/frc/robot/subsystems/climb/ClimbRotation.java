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

import com.ma5951.utils.JoystickContainer;
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
        rotationPID.setF(2.5);
        shuffleboard = new Shuffleboard("ClimbRotation");
        rotationPID.setIntegratorRange(-0.15, 0.15);
    }

    public static ClimbRotation getInstance() {
        if (climbRotation == null) {
            climbRotation = new ClimbRotation();
        }
        return climbRotation;
    }

    public void setF(double F) {
        rotationPID.setF(F);
        rotationPID.reset();
    }

    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        rotationPID.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    public double averageDis() {
        return (leftRotationMotor.getSelectedSensorPosition() + rightRotationMotor.getSelectedSensorPosition()) / 2;
    }

    public double getAngle() {
        return (averageDis() * 90) / ClimbConstants.TICK_FOR_90_DEGREES_ROTATION;
    }

    public boolean getHallEffect() {
        return !hallEffect.get();
    }

    @Override
    public void setSetpoint(double setPoint) {
        rotationPID.setSetpoint((setPoint / 90) * ClimbConstants.TICK_FOR_90_DEGREES_ROTATION);
    }

    @Override
    public boolean atSetpoint() {
        return rotationPID.atSetpoint();
    }

    @Override
    public double calculate() {
        return rotationPID.calculate(averageDis());
    }

    @Override
    public void setVoltage(double voltage) {
        leftRotationMotor.set(TalonFXControlMode.PercentOutput, 0);//leftRotationMotor.set(TalonFXControlMode.PercentOutput, voltage / 12.0);
    }

    public double getVoltage() {
        return leftRotationMotor.getMotorOutputVoltage();
    }

    public double getCurrent() {
        return (leftRotationMotor.getStatorCurrent() + rightRotationMotor.getStatorCurrent()) / 2.0;
    }

    public boolean canMovePassive() {
        return (averageDis() * 90) / ClimbConstants.TICK_FOR_90_DEGREES_ROTATION <= -2.9;
    }

    public boolean canMove() {
        return getCurrent() > 20;
    }

    public void reset() {
        leftRotationMotor.setSelectedSensorPosition(0);
        rightRotationMotor.setSelectedSensorPosition(0);
    }

    public void setOutputRange(double low, double high) {
        rotationPID.setOutputRange(low, high);
    }

    @Override
    public void periodic() {
        if (getHallEffect() && leftRotationMotor.getMotorOutputPercent() == 0) {
            // reset();
        }
        shuffleboard.addBoolean("hallEffect", getHallEffect());
        shuffleboard.addBoolean("open passive?", canMovePassive());
        shuffleboard.addBoolean("At setpoint", rotationPID.atSetpoint());

        // This method will be called once per scheduler run
        shuffleboard.addNum("pid", calculate());
        shuffleboard.addNum("averageDis", (averageDis() * 90) / ClimbConstants.TICK_FOR_90_DEGREES_ROTATION);
        shuffleboard.addNum("setPoint", (rotationPID.getSetpoint() * 90) / ClimbConstants.TICK_FOR_90_DEGREES_ROTATION);
        /*
         * rotationPID.setP(shuffleboard.getNum("KP"));
         * rotationPID.setI(shuffleboard.getNum("KI"));
         * rotationPID.setD(shuffleboard.getNum("KD"));
         * setSetpoint(shuffleboard.getNum("setSetPoint"));
         * if(shuffleboard.getBoolean("Start")){
         * setVoltage(calculate());
         * }else{
         * setVoltage(JoystickContainer.operatingJoystick.getRawAxis(4) * 12 * 0.4);
         * }
         */
    }
}
