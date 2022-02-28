// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.Shuffleboard;
import com.ma5951.utils.controllers.PIDController;
import com.ma5951.utils.motor.MA_SparkMax;
import com.ma5951.utils.subsystem.ControlSubsystem;

public class ClimbExtension extends SubsystemBase implements ControlSubsystem {
  /** Climb Extension Arm */
  private static ClimbExtension climbExtension;
  private MA_SparkMax extensionMotor;
  private PIDController extensionPID;
  private Shuffleboard shuffleboard;

  public ClimbExtension() {
    extensionMotor = new MA_SparkMax(PortMap.climbExtensionMotor, false, RobotConstants.KMOTOR_BRAKE,
        RobotConstants.ENCODER.Alternate_Encoder, CANSparkMaxLowLevel.MotorType.kBrushless);
    extensionPID = new PIDController(ClimbConstants.EXTENSION_KP, ClimbConstants.EXTENSION_KI,
        ClimbConstants.EXTENSION_KD, 0, ClimbConstants.EXTENSION_TOLERANCE, -12, 12);
    shuffleboard = new Shuffleboard("ClimbExtension");

    extensionMotor.resetEncoder();
  }

  public static ClimbExtension getInstance() {
    if (climbExtension == null) {
      climbExtension = new ClimbExtension();
    }
    return climbExtension;
  }

  @Override
  public void setSetpoint(double setPoint) {
    extensionPID.setSetpoint(setPoint);
  }

  @Override
  public boolean atSetpoint() {
    return extensionPID.atSetpoint();
  }

  @Override
  public double calculate() {
    return extensionPID.calculate(extensionMotor.getPosition());
  }

  @Override
  public void setVoltage(double voltage) {
    extensionMotor.setVoltage(voltage);
  }

  public boolean canMove() {
    return true; // TODO
  }

  @Override
  public void reset() {
  }

  public double getVoltage() {
    return extensionMotor.getOutput() * 12;
  }

  public double getCurrent() {
    return extensionMotor.getStatorCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleboard.addNum("encoder", extensionMotor.getPosition());
  }
}
