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

public class ClimbPassive extends SubsystemBase implements ControlSubsystem {
  /** Climb Passive Arms */
  private static ClimbPassive climbPassive;
  private MA_SparkMax leftPassiveMotor, rightPassiveMotor;
  private PIDController passivePID;

  public ClimbPassive() {
    leftPassiveMotor = new MA_SparkMax(PortMap.climbPassiveMotor, false, RobotConstants.KMOTOR_BRAKE,
        RobotConstants.ENCODER.Encoder, CANSparkMaxLowLevel.MotorType.kBrushless);

    rightPassiveMotor = new MA_SparkMax(RobotConstants.ID10, false, RobotConstants.KMOTOR_BRAKE,
        RobotConstants.ENCODER.Encoder, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightPassiveMotor.follow(leftPassiveMotor);

    passivePID = new PIDController(ClimbConstants.PASSIVE_KP, ClimbConstants.PASSIVE_KI, ClimbConstants.PASSIVE_KD,
        ClimbConstants.PASSIVE_TOLERANCE);
  }

  public static ClimbPassive getInstance() {
    if (climbPassive == null) {
      climbPassive = new ClimbPassive();
    }
    return climbPassive;
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
    leftPassiveMotor.setVoltage(voltage);
  }

  public double getVoltage() {
    return leftPassiveMotor.getOutput() * 12;
  }

  public boolean canMove(){
    return true; //TODO
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
