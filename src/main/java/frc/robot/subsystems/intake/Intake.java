// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.motor.Piston;
import com.ma5951.utils.motor.MA_TalonSRX;
import com.ma5951.utils.subsystem.MotorSubsystem;
import com.ma5951.utils.subsystem.PistonSubsystem;

public class Intake extends SubsystemBase implements MotorSubsystem, PistonSubsystem {
  /** Creates a new Instake. */
  private MA_TalonSRX intakeMotor;
  private Piston intakePiston;
  private static Intake intake;

  public Intake() {
    intakeMotor = new MA_TalonSRX(PortMap.intakeMotor, false, RobotConstants.KMOTOR_COAST);
    intakePiston = new Piston(PortMap.intakePistonForward, PortMap.intakePistonReverse);
  }

  public void open() {
    intakePiston.set(true);
  }

  public void close() {
    intakePiston.set(false);
  }

  public boolean isOpen() {
    return intakePiston.get();
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

  public boolean canMove() {
    return true;
  }

  @Override
  public void off() {
    intakePiston.off();
  }

  @Override
  public void reset() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
