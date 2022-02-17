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

public class ClimbExtension extends SubsystemBase implements ControlInterfaceSubsystem {
  /** Climb Extension Arm */
  private MASparkMax extensionMotor;
  private MAPidController extensionPID;

  public ClimbExtension() {
    extensionMotor = new MASparkMax(PortMap.climbExtentionMotor, false, RobotConstants.KMOTOR_BRAKE, RobotConstants.ENCODER.Alternate_Encoder, CANSparkMaxLowLevel.MotorType.kBrushless);
    extensionPID = new MAPidController(ClimbConstants.EXTENSION_KP, ClimbConstants.EXTENSION_KI, ClimbConstants.EXTENSION_KD, 0, 30, -12, 12);
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
    extensionMotor.setvoltage(voltage);
  }

  @Override
  public double getVoltage() {
    return extensionMotor.getOutput() * 12;
  }

  public double getCurrent() {
    return extensionMotor.getStatorCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
