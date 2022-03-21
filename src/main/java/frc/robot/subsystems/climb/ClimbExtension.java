// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

import com.ma5951.utils.JoystickContainer;
import com.ma5951.utils.Shuffleboard;
import com.ma5951.utils.controllers.PIDController;
import com.ma5951.utils.subsystem.ControlSubsystem;

public class ClimbExtension extends SubsystemBase implements ControlSubsystem {
  /** Climb Extension Arm */
  private static ClimbExtension climbExtension;
  private TalonFX extensionMotor;
  private PIDController extensionPID;
  private Shuffleboard shuffleboard;


  public ClimbExtension() {
    extensionMotor = new TalonFX(PortMap.climbExtensionMotor);
    extensionMotor.setNeutralMode(NeutralMode.Brake);
    
    extensionPID = new PIDController(ClimbConstants.EXTENSION_KP, ClimbConstants.EXTENSION_KI,
        ClimbConstants.EXTENSION_KD, 0, ClimbConstants.EXTENSION_TOLERANCE, -12, 12);
    shuffleboard = new Shuffleboard("ClimbExtension");
     reset();
     
  }

  public static ClimbExtension getInstance() {
    if (climbExtension == null) {
      climbExtension = new ClimbExtension();
    }
    return climbExtension;
  }

  @Override
  public void setSetpoint(double setPoint) {
    extensionPID.setSetpoint(setPoint * ClimbConstants.TICK_PER_METER_EXTENSION);
  }
  
  public double getSetPoint(){
    return extensionPID.getSetpoint();
  }

  @Override
  public boolean atSetpoint() {
    return extensionPID.atSetpoint();
  }

  @Override
  public double calculate() {
    return extensionPID.calculate(extensionMotor.getSelectedSensorPosition());
  }

  @Override
  public void setVoltage(double voltage) {
    extensionMotor.set(TalonFXControlMode.PercentOutput, voltage / 12.0);
  }

  public boolean canMove() {
    return extensionMotor.getSelectedSensorPosition() > ClimbConstants.MAX_POSITION;
  }

  public void keepArmInPlace() {
    extensionPID.setSetpoint(extensionMotor.getSelectedSensorPosition() *  ClimbConstants.TICK_PER_METER_EXTENSION);
  }

  public void reset() {
    extensionMotor.setSelectedSensorPosition(0);
  }
public double getDistance(){
  return extensionMotor.getSelectedSensorPosition();
}

  public double getVoltage() {
    return extensionMotor.getMotorOutputVoltage();
  }

  public double getCurrent() {
    return extensionMotor.getStatorCurrent();
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleboard.addNum("encoder", extensionMotor.getSelectedSensorPosition()/ ClimbConstants.TICK_PER_METER_EXTENSION);
    shuffleboard.addBoolean("setpoint", extensionPID.atSetpoint());
    shuffleboard.addNum("pid value", calculate());
    shuffleboard.addNum("Current", getCurrent());
    shuffleboard.addNum("getSetPoint", extensionPID.getSetpoint() / ClimbConstants.TICK_PER_METER_EXTENSION);
/*
    extensionPID.setP(shuffleboard.getNum("KP"));
    extensionPID.setI(shuffleboard.getNum("KI"));
    extensionPID.setD(shuffleboard.getNum("KD"));
    setSetpoint(shuffleboard.getNum("setSetPoint"));
    if(shuffleboard.getBoolean("Start")){
        setVoltage(calculate());
    }else{
        
      setVoltage(JoystickContainer.operatingJoystick.getRawAxis(1) * 12 * 0.4);
  }
  */
}
}
