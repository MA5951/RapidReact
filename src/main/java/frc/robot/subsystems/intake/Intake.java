// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.motor.MAMotorControllerInterface;
import frc.robot.utils.motor.MAPiston;
import frc.robot.utils.motor.MATalonSRX;
import frc.robot.utils.subsystem.MotorInterfaceSubsystem;
import frc.robot.utils.subsystem.PistonInterfaceSubsystem;

public class Intake extends SubsystemBase implements MotorInterfaceSubsystem, PistonInterfaceSubsystem {
  /** Creates a new Instake. */
  private MAMotorControllerInterface intakeMotor;
  private MAPiston piston2;
  private MAPiston piston1;
  private static Intake intake;

  public Intake() {
    intakeMotor = new MATalonSRX(RobotConstants.ID5, false, RobotConstants.KMOTOR_COAST);
    piston2 = new MAPiston(0,1);
    piston1 = new MAPiston(7,4);
  }

  public void open(){
    piston1.set(true);
    piston2.set(true);
  }

  public void close(){
    piston1.set(false);
    piston2.set(false);
  }

  public boolean isOpen(){
    return piston1.get();
  }

  public void setVoltage (double voltege){
    intakeMotor.setvoltage(voltege);
  }

  public double getVoltage(){
    return intake.getVoltage();
  }


  public static Intake getinstance() {
    if (intake == null){
      intake = new Intake();
    }
    return intake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
