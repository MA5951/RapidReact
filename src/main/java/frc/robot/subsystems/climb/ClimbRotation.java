// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.controllers.MAPidController;
import frc.robot.utils.motor.MASparkMax;
import frc.robot.utils.subsystem.ControlInterfaceSubsystem;

public class ClimbRotation extends SubsystemBase implements ControlInterfaceSubsystem {
    /**
     * Climb Rotation Arm
     */
    private MASparkMax leftRotationMotor, rightRotationMotor;
    private MAPidController rotationPID;

    public ClimbRotation() {
        leftRotationMotor = new MASparkMax(PortMap.climbRotationLeftMotor, false, RobotConstants.KMOTOR_BRAKE, RobotConstants.LIMIT_SWITCH.forward, RobotConstants.ENCODER.Alternate_Encoder, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftRotationMotor.enableLimitSwitchF(true);

        rightRotationMotor = new MASparkMax(PortMap.climbRotationRightMotor, false, RobotConstants.KMOTOR_BRAKE, RobotConstants.ENCODER.Alternate_Encoder, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightRotationMotor.follow(leftRotationMotor);

        rotationPID = new MAPidController(ClimbConstants.ROTAION_KP, ClimbConstants.ROTAION_KI, ClimbConstants.ROTAION_KD, 0, 0, -12, 12);

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
        leftRotationMotor.setvoltage(voltage);
    }

    @Override
    public double getVoltage() {
        return leftRotationMotor.getOutput() * 12;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
