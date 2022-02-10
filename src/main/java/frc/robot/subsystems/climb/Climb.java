// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.RobotConstants.ENCODER;
import frc.robot.utils.RobotConstants.LIMIT_SWITCH;
import frc.robot.utils.controllers.MAPidController;
import frc.robot.utils.motor.MAPiston;
import frc.robot.utils.motor.MASparkMax;
import frc.robot.utils.subsystem.PistonInterfaceSubsystem;

public class Climb extends SubsystemBase implements PistonInterfaceSubsystem {
  /** Creates a new Climb. */
  private MASparkMax rotationMotor, extensionMotor;
  private MAPidController rotationMotionPID, extensionMotionPID;
  private MAPiston leftPiston, rightPiston;

  private static Climb climb;

  public Climb() {
    rotationMotor = new MASparkMax(RobotConstants.ID6, false, RobotConstants.KMOTOR_BRAKE, LIMIT_SWITCH.forward,
        ENCODER.Alternate_Encoder, MotorType.kBrushless);
    rotationMotor.enableLimitSwitchF(true);

    extensionMotor = new MASparkMax(RobotConstants.ID7, false, RobotConstants.KMOTOR_BRAKE, ENCODER.Alternate_Encoder, MotorType.kBrushless);
    
    rotationMotionPID = new MAPidController(ClimbConstants.ROTAION_KP, ClimbConstants.ROTAION_KI, ClimbConstants.ROTAION_KD, 0, 0, -12, 12);
    extensionMotionPID = new MAPidController(ClimbConstants.EXTENSION_KP, ClimbConstants.EXTENSION_KI, ClimbConstants.EXTENSION_KD, 0, 30, -12, 12);

    leftPiston = new MAPiston(2, 3);
    rightPiston = new MAPiston(8, 9);
  }

  public void setRotationVoltage(double voltage) {
    rotationMotor.setvoltage(voltage);
  }

  public void setRotationPower(double power) {
    setRotationVoltage(power * 12);
  }

  public void setExtensionVoltage(double voltage) {
    extensionMotor.setvoltage(voltage);
  }

  public void setExtentionPower(double power) {
    setExtensionVoltage(power * 12);
  }

  public void resetRotationEncoder() {
    rotationMotor.resetEncoder();
  }

  public void resetExtensionEncoder() {
    extensionMotor.resetEncoder();
  }

  public boolean getRotateHallEffect() {
    return rotationMotor.getForwardLimitSwitch();
  }

  public void setRotationSetpoint(double setpoint) {
    rotationMotionPID.setF(((setpoint / RobotConstants.KMAX_RPM_NEO) * 12) * 1.1);
    rotationMotionPID.setSetpoint(setpoint);
  }

  public void setExtensionSetpoint(double setpoint) {
    extensionMotionPID.setF(((setpoint / RobotConstants.KMAX_RPM_NEO) * 12) * 1.1);
    extensionMotionPID.setSetpoint(setpoint);
  }

  public boolean setRotationAtSetpoint() {
    return rotationMotionPID.atSetpoint();
  }

  public boolean setExtensionatSetpoint() {
    return extensionMotionPID.atSetpoint();
  }

  public double calculateRotation(double input) {
    return rotationMotionPID.calculate(input);
  }

  public double calculateExtension(double input) {
    return extensionMotionPID.calculate(input);
  }

  public void open() {
    leftPiston.set(true);
    rightPiston.set(true);
  }

  public void close() {
    leftPiston.set(false);
    rightPiston.set(true);
  }

  public boolean isOpen() {
    return leftPiston.get() && rightPiston.get();
  }

  public static Climb getinstance() {
    if (climb == null) {
      climb = new Climb();
    }
    return climb;
  }

  @Override
  public void periodic() {
  }
}
