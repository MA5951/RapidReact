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

  private Piston shooterPiston;

  private PIDController pidController;

  private Shuffleboard shooterShuffleboard;

  private static Shooter shooter;

  public Shooter() {
    shooterLeftMotor = new MA_SparkMax(PortMap.shooterLeftMotor, true, false, ENCODER.Encoder, MotorType.kBrushless); // ID8
    shooterRightMotor = new MA_SparkMax(PortMap.shooterRightMotor, false, false, ENCODER.Encoder, MotorType.kBrushless); // ID9

    shooterPiston = new Piston(PortMap.shooterPistonForward, PortMap.shooterPistonReverse);

    pidController = new PIDController(ShooterConstants.SHOOTER_VELOCITY_PID_KP,
        ShooterConstants.SHOOTER_VELOCITY_PID_KI, ShooterConstants.SHOOTER_VELOCITY_PID_KD, 0,
        ShooterConstants.SHOOTER_VELOCITY_PID_TOLERANCE, -12, 12);

    shooterShuffleboard = new Shuffleboard(ShooterConstants.SYSTEM_NAME);

    shooterRightMotor.follow(shooterLeftMotor);
  }

  public void setMotor(double power) {
    shooterLeftMotor.setVoltage(power);
  }

  public double getVelocity() {
    return (shooterLeftMotor.getVelocity() + shooterRightMotor.getVelocity()) / 2.0;
  }

  public void setSetpoint(double setpoint) {
    pidController.setF(((pidController.getSetpoint() / RobotConstants.KMAX_RPM_NEO) * 12)
        * ShooterConstants.SHOOTER_VELOCITY_PID_KF);
    pidController.setSetpoint(setpoint);
  }

  public double calculate(double input) {
    return pidController.calculate(input);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void open() {
    shooterPiston.set(true);
  }

  public void close() {
    shooterPiston.set(false);
  }

  public boolean isOpen() {
    return shooterPiston.get();
  }

  public static Shooter getinstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }

  @Override
  public void periodic() {
    shooterShuffleboard.addNum("Shooter RPM", getVelocity());
    shooterShuffleboard.addBoolean("At Setpoint", atSetpoint());
  }
}
