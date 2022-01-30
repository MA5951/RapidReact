// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MAUtils2.RobotConstants;
import frc.robot.MAUtils2.MAMotorController.MAMotorControlInterface;
import frc.robot.MAUtils2.MAMotorController.MAPiston;
import frc.robot.MAUtils2.MAMotorController.MAVictorSPX;
import frc.robot.MAUtils2.MASubsystem.MotorInterfaceSubsystem;
import frc.robot.MAUtils2.MASubsystem.PistonInterfaceSubsystem;

public class Intake extends SubsystemBase implements MotorInterfaceSubsystem, PistonInterfaceSubsystem {
  /** Creates a new Instake. */
  private MAMotorControlInterface intakeMotor;
  private MAPiston piston;
  private static Intake intake;

  public Intake() {
    intakeMotor = new MAVictorSPX(RobotConstants.ID5, false, false);
    piston = new MAPiston(1, 2);
  }

  public void open(){
    piston.set(true);
  }

  public void close(){
    piston.set(false);
  }

  public boolean isOpen(){
    return piston.get();
  }

  public void setVoltege (double voltege){
    intake.setVoltege(voltege);
  }

  public double getVoltege(){
    return intake.getVoltege();
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
