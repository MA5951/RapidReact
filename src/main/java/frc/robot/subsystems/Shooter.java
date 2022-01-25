// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MAUtils2.MAShuffleboard;
import frc.robot.MAUtils2.RobotConstants;
import frc.robot.MAUtils2.MAMotorController.MASparkMax;
import frc.robot.MAUtils2.RobotConstants.ENCODER;
import frc.robot.MAUtils2.controllers.MAPidController;
import frc.robot.commands.Shooter.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private MASparkMax shooterAMotor;
  private MASparkMax shooterBMotor;

  private MAPidController pidController;

  private MAShuffleboard shooterShuffleboard;

  private static Shooter shooter;

  public Shooter() {
    shooterAMotor = new MASparkMax(RobotConstants.ID6, true, 0, false, false, false, ENCODER.Alternate_Encoder, MotorType.kBrushless);
    shooterBMotor = new MASparkMax(RobotConstants.ID7, false, 0, false, false, false, ENCODER.Alternate_Encoder, MotorType.kBrushless);

    pidController = new MAPidController(ShooterConstants.SHOOTER_VELOCITY_PID_KP, ShooterConstants.SHOOTER_VELOCITY_PID_KI, ShooterConstants.SHOOTER_VELOCITY_PID_KD, 0, 80, -12, 12);

    shooterShuffleboard = new MAShuffleboard(ShooterConstants.SYSTEM_NAME);

    shooterBMotor.follow(shooterAMotor.getCanSparkMax());
  }

  public void setMotor(double power){
    shooterAMotor.set(power);
  }

  public double getEncoder(){
    return shooterAMotor.getVelocity();
  }

  public void setSetpoint(double setpoint){
    pidController.setSetpoint(setpoint);
  }

  public double calculate(double input){
    pidController.setF((pidController.getSetpoint() / RobotConstants.KMAX_RPM_NEO) * 12);
    return pidController.calculate(input);
  }

  public boolean atSetpoint(){
    return pidController.atSetpoint();
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
