// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import com.ma5951.utils.Shuffleboard;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.RobotConstants.ENCODER;
import com.ma5951.utils.controllers.PIDController;
import com.ma5951.utils.motor.Piston;
import com.ma5951.utils.motor.MA_SparkMax;
import com.ma5951.utils.subsystem.PistonSubsystem;

public class Shooter extends SubsystemBase implements PistonSubsystem {
  /** Creates a new Shooter. */
  private MA_SparkMax shooterLeftMotor;
  private MA_SparkMax shooterRightMotor;

  private Piston shooterLeftPiston;
  private Piston shooterRightPiston;

  private PIDController pidController;

  private Shuffleboard shooterShuffleboard;

  private static Shooter shooter;

  public Shooter() {
    shooterLeftMotor = new MA_SparkMax(PortMap.shooterLeftMotor, true, false, ENCODER.Encoder, MotorType.kBrushless); //ID8
    shooterRightMotor = new MA_SparkMax(PortMap.shooterRightMotor, false, false, ENCODER.Encoder, MotorType.kBrushless); //ID9

    shooterLeftPiston = new Piston(PortMap.shooterLeftPistonForward, PortMap.shooterLeftPistonReverse);
    shooterRightPiston = new Piston(PortMap.shooterRightPistonForward, PortMap.shooterRightPistonReverse);

    pidController = new PIDController(ShooterConstants.SHOOTER_VELOCITY_PID_KP, ShooterConstants.SHOOTER_VELOCITY_PID_KI, ShooterConstants.SHOOTER_VELOCITY_PID_KD, 0, 50, -12, 12);

    shooterShuffleboard = new Shuffleboard(ShooterConstants.SYSTEM_NAME);

    shooterRightMotor.follow(shooterLeftMotor);
  }

  public void setMotor(double power){
    shooterLeftMotor.setVoltage(power);
  }

  public double getEncoder(){
    return shooterLeftMotor.getVelocity();
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
    shooterLeftPiston.set(true);
    shooterRightPiston.set(true);
  }

  public void close(){
    shooterLeftPiston.set(false);
    shooterRightPiston.set(false);
  }

  public boolean isOpen(){
    return shooterLeftPiston.get();
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
