// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.motor.MA_TalonSRX;
import com.ma5951.utils.subsystem.MotorSubsystem;
import com.ma5951.utils.subsystem.PistonSubsystem;

public class Intake extends SubsystemBase implements MotorSubsystem, PistonSubsystem {
  /** Creates a new Instake. */
  private MA_TalonSRX intakeMotor;
  private DoubleSolenoid intakePiston;
  private static Intake intake;

  public Intake() {
    intakeMotor = new MA_TalonSRX(PortMap.intakeMotor, false, RobotConstants.KMOTOR_COAST);
    intakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, PortMap.intakePistonForward, PortMap.intakePistonReverse);
  }

  public void open() {
    intakePiston.set(Value.kForward);
  }

  public void close() {
    intakePiston.set(Value.kReverse);
  }

  public boolean isOpen() {
    return intakePiston.get() == Value.kForward;
  }

  public void setVoltage(double voltege) {
    intakeMotor.setVoltage(voltege);
  }

  public double getVoltage() {
    return intake.getVoltage();
  }

  public static Intake getInstance() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }

  public boolean canMove() {
    return true;
  }

  public void off() {
    intakePiston.set(Value.kReverse);
  }

  public void reset() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
