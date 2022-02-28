// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import com.ma5951.utils.Shuffleboard;
import frc.robot.Limelight;

import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;
import com.ma5951.utils.Calculations;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.RobotConstants.ENCODER;
import com.ma5951.utils.controllers.PIDController;
import com.ma5951.utils.motor.Piston;
import com.ma5951.utils.motor.MA_SparkMax;
import com.ma5951.utils.subsystem.ControlSubsystem;
import com.ma5951.utils.subsystem.MotorSubsystem;
import com.ma5951.utils.subsystem.PistonSubsystem;

public class Shooter extends SubsystemBase implements PistonSubsystem, ControlSubsystem {
  /** Creates a new Shooter. */
  private MA_SparkMax shooterLeftMotor;
  private MA_SparkMax shooterRightMotor;

  private DoubleSolenoid shooterPiston;

  private PIDController pidController;

  private Shuffleboard shooterShuffleboard;

  private static Shooter shooter;


  public Shooter() {
    shooterLeftMotor = new MA_SparkMax(PortMap.shooterLeftMotor, true, false, ENCODER.Encoder, MotorType.kBrushless); // ID8
    shooterRightMotor = new MA_SparkMax(PortMap.shooterRightMotor, false, false, ENCODER.Encoder, MotorType.kBrushless); // ID9

    shooterPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, PortMap.shooterPistonForward, PortMap.shooterPistonReverse);//Piston(PortMap.shooterPistonForward, PortMap.shooterPistonReverse);

    pidController = new PIDController(ShooterConstants.SHOOTER_VELOCITY_PID_KP,
        ShooterConstants.SHOOTER_VELOCITY_PID_KI, ShooterConstants.SHOOTER_VELOCITY_PID_KD, 0,
        ShooterConstants.SHOOTER_VELOCITY_PID_TOLERANCE, -12, 12);

    shooterShuffleboard = new Shuffleboard(ShooterConstants.SYSTEM_NAME);

    shooterRightMotor.follow(shooterLeftMotor);

    shooterLeftMotor.resetEncoder();
    shooterRightMotor.resetEncoder();

    // shooterLeftMotor.setInverted(true);
    // shooterRightMotor.setInverted(true);
  }

  public void resetPID() {
    pidController.reset();
  }

  public void setVoltage(double power) {
    shooterLeftMotor.setVoltage(power);
  }

  public double getVelocity() {
    return (shooterLeftMotor.getVelocity() + shooterRightMotor.getVelocity()) / 2.0;
  }

  public void setSetpoint(double setpoint) {
    pidController.setSetpoint(setpoint);
    pidController.setF(((setpoint / RobotConstants.KMAX_RPM_NEO) * 12)
        * ShooterConstants.SHOOTER_VELOCITY_PID_KF);
  }

  public double calculate() {
    return pidController.calculate(getVelocity());
  }
  
public double calculate(double input) {
    return pidController.calculate(input);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public double calculateVelocityMPS() {
    double dx = Limelight.distance();
    dx = 3.0; // TODO
    double radiansAngle = Math.toRadians(ShooterConstants.SHOOTER_ANGLE);
    return Math.sqrt(
      ((-4.9 * Math.pow(dx, 2))) / 
      ((ShooterConstants.K_DELTA_Y - (dx * Math.tan(radiansAngle))) * Math.pow(Math.cos(radiansAngle), 2))
    );
  }

  public double getRPMVelocity() {  
    double angularVelocity = calculateVelocityMPS() / 0.12;
    return (angularVelocity * 60) / (2 * Math.PI);
  }

  public void open() {
    shooterPiston.set(Value.kForward);
  }

  public void close() {
    shooterPiston.set(Value.kReverse);
  }

  public boolean isOpen() {
    return shooterPiston.get() == Value.kForward;
  }

  public static Shooter getinstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }

  public void reset() {
  }

  public boolean canMove() {
    return true;
  }

  public double getStator(){
    return shooterRightMotor.getStatorCurrent();
  }

  public double getShooterPower(){
    return ((107.98 * Math.pow(frc.robot.Limelight.distance(), 2) 
        - 467.32 * frc.robot.Limelight.distance() + 3055.2) * -1) * 1.05;
  }

  public double getVoltage(){
    return shooterLeftMotor.getOutput();
  }

  @Override
  public void periodic() {
    shooterShuffleboard.addNum("Current", shooterRightMotor.getStatorCurrent());
    shooterShuffleboard.addNum("pid value", pidController.calculate(getVelocity()));
    shooterShuffleboard.addNum("shooter", shooterLeftMotor.getPosition());
    shooterShuffleboard.addNum("Shooter RPM", getVelocity());
    shooterShuffleboard.addBoolean("At Setpoint", atSetpoint());
    shooterShuffleboard.addNum("RPM Setpoint", getRPMVelocity());
    shooterShuffleboard.addNum("Shooter Calc", getShooterPower());
  }

  public void off() {
    shooterPiston.set(Value.kReverse);
  }
}
