// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAShuffleboard;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.RobotConstants.ENCODER;
import frc.robot.utils.controllers.MAPidController;
import frc.robot.utils.motor.MAPiston;
import frc.robot.utils.motor.MASparkMax;
import frc.robot.utils.subsystem.PistonInterfaceSubsystem;

public class Shooter extends SubsystemBase implements PistonInterfaceSubsystem {
  /** Creates a new Shooter. */
  private MASparkMax shooterAMotor;
  private MASparkMax shooterBMotor;

  private MAPiston shooterAPiston;

  private MAPidController pidController;

  private MAShuffleboard shooterShuffleboard;

  private static Shooter shooter;

  public Shooter() {
    shooterAMotor = new MASparkMax(RobotConstants.ID8, true, false, ENCODER.Encoder, MotorType.kBrushless); //ID8
    shooterBMotor = new MASparkMax(RobotConstants.ID9, false, false, ENCODER.Encoder, MotorType.kBrushless); //ID9

    shooterAPiston = new MAPiston(5, 6);

    pidController = new MAPidController(ShooterConstants.SHOOTER_VELOCITY_PID_KP, ShooterConstants.SHOOTER_VELOCITY_PID_KI, ShooterConstants.SHOOTER_VELOCITY_PID_KD, 0, 50, -12, 12);

    shooterShuffleboard = new MAShuffleboard(ShooterConstants.SYSTEM_NAME);

    shooterBMotor.follow(shooterAMotor.getCanSparkMax());
  }

  public void setMotor(double power){
    shooterAMotor.setvoltage(power);
  }

  public double getEncoder(){
    return shooterAMotor.getVelocity();
  }

  public void setSetpoint(double setpoint){
    pidController.setF(((pidController.getSetpoint() / RobotConstants.KMAX_RPM_NEO) * 12) * 1.1);
    pidController.setSetpoint(setpoint);
  }

  public double calculate(double input){
    return pidController.calculate(input);
  }

  public boolean atSetpoint(){
    return pidController.atSetpoint();
  }

  public void open(){
    shooterAPiston.set(true);
  }

  public void close(){
    shooterAPiston.set(false);
  }

  public boolean isOpen(){
    return shooterAPiston.get();
  }

  public static Shooter getinstance(){
    if (shooter == null){
      shooter = new Shooter();
    }
    return shooter;
  }

  @Override
  public void periodic() {
    shooterShuffleboard.addNum("Shooter RPM", getEncoder());
    shooterShuffleboard.addBoolean("At Setpoint", atSetpoint());
  }
}
