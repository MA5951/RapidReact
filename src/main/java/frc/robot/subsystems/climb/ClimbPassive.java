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

public class ClimbPassive extends SubsystemBase implements ControlInterfaceSubsystem {
  /** Climb Passive Arms */
  private MASparkMax leftPassiveMotor, rightPassiveMotor;
  private MAPidController passivePID;

  public ClimbPassive() {
    leftPassiveMotor = new MASparkMax(PortMap.climbPassiveMotor, false, RobotConstants.KMOTOR_BRAKE, RobotConstants.ENCODER.Encoder, CANSparkMaxLowLevel.MotorType.kBrushless);

    rightPassiveMotor = new MASparkMax(RobotConstants.ID10, false, RobotConstants.KMOTOR_BRAKE, RobotConstants.ENCODER.Encoder, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightPassiveMotor.follow(leftPassiveMotor);

    passivePID = new MAPidController(ClimbConstants.PASSIVE_KP, ClimbConstants.PASSIVE_KI, ClimbConstants.PASSIVE_KD, 20);
  }

  @Override
  public void setSetpoint(double setPoint) {
    passivePID.setSetpoint(setPoint);
  }

  @Override
  public boolean atSetpoint() {
    return passivePID.atSetpoint();
  }

  @Override
  public double calculate() {
    return leftPassiveMotor.getPosition();
  }

  @Override
  public void setVoltage(double voltage) {
    leftPassiveMotor.setvoltage(voltage);
  }

  @Override
  public double getVoltage() {
    return leftPassiveMotor.getOutput() * 12;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
