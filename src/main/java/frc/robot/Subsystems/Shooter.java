// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.Shooter.ShooterConstants;
import frc.robot.MAUtils2.RobotConstants;
import frc.robot.MAUtils2.MAMotorController.MASparkMax;
import frc.robot.MAUtils2.RobotConstants.ENCODER;
import frc.robot.MAUtils2.controllers.MAPidController;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private MASparkMax shooterAMotor;
  private MASparkMax shooterBMotor;

  private MAPidController pidController;

  private static Shooter shooter;

  public Shooter() {
    shooterAMotor = new MASparkMax(RobotConstants.ID6, false, 0, false, false, false, ENCODER.Encoder.Alternate_Encoder, MotorType.kBrushless);
    shooterBMotor = new MASparkMax(RobotConstants.ID7, false, 0, false, false, false, ENCODER.Encoder.Alternate_Encoder, MotorType.kBrushless);

    pidController = new MAPidController(ShooterConstants.SHOOTER_VELOCITY_PID_KP, ShooterConstants.SHOOTER_VELOCITY_PID_KI, ShooterConstants.SHOOTER_VELOCITY_PID_KD, 0, 0, -12, 12);

    //shooterBMotor.follow(shooterAMotor); //TODO: Fix the follow function that MASparkmax will follow another MASparkmax
  }

  public void setMotor(double power){
    shooterAMotor.set(power);
  }

  public double getEncoder(){
    return shooterAMotor.getVelocity();
  }

  public double calculate(double input){
    return pidController.calculate(input);
  }

  public static Shooter getinstance(){
    if (shooter == null){
      shooter = new Shooter();
    }
    return shooter;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
