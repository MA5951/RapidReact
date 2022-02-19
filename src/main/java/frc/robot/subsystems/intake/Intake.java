// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.motor.MotorController;
import com.ma5951.utils.motor.Piston;
import com.ma5951.utils.motor.MA_TalonSRX;
import com.ma5951.utils.subsystem.MotorSubsystem;
import com.ma5951.utils.subsystem.PistonSubsystem;

public class Intake extends SubsystemBase implements MotorSubsystem, PistonSubsystem {
  /** Creates a new Instake. */
  private MA_TalonSRX intakeMotor;
  private Piston intakeLeftPiston;
  private Piston intakeRightPiston;
  private static Intake intake;

  public Intake() {
    intakeMotor = new MA_TalonSRX(PortMap.intakeMotor, false, RobotConstants.KMOTOR_COAST);
    intakeLeftPiston = new Piston(PortMap.intakeLeftPistonForward, PortMap.intakeLeftPistonReverse);
    intakeRightPiston = new Piston(PortMap.intakeRightPistonForward, PortMap.intakeRightPistonReverse);
  }

  public void open() {
    intakeRightPiston.set(true);
    intakeLeftPiston.set(true);
  }

  public void close() {
    intakeRightPiston.set(false);
    intakeLeftPiston.set(false);
  }

  public boolean isOpen() {
    return intakeRightPiston.get();
  }

  public void setVoltage(double voltege) {
    intakeMotor.setVoltage(voltege);
  }

  public double getVoltage() {
    return intake.getVoltage();
  }

  public static Intake getinstance() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
