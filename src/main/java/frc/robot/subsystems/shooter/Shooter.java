// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight;
import frc.robot.PortMap;
import com.ma5951.utils.Shuffleboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.RobotConstants.ENCODER;
import com.ma5951.utils.controllers.PIDController;
import com.ma5951.utils.motor.MA_SparkMax;
import com.ma5951.utils.subsystem.ControlSubsystem;
import com.ma5951.utils.subsystem.PistonSubsystem;

public class Shooter extends SubsystemBase implements PistonSubsystem, ControlSubsystem {
  /** Creates a new Shooter. */
  private TalonFX shooterLeftMotor;
  private TalonFX shooterRightMotor;

  private DoubleSolenoid shooterPiston;

  private PIDController pidController;

  private Shuffleboard shooterShuffleboard;

  private static Shooter shooter;
  // private static final String sd = "from distance shotting b";
  // private static final String s = "shotting b";
  // private static final String a = "angle b";

  public Shooter() {
    shooterLeftMotor = new TalonFX(PortMap.shooterLeftMotor); 
    shooterRightMotor = new TalonFX(PortMap.shooterRightMotor); 


    shooterPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, PortMap.shooterPistonForward,
        PortMap.shooterPistonReverse);// Piston(PortMap.shooterPistonForward, PortMap.shooterPistonReverse);

    pidController = new PIDController(ShooterConstants.SHOOTER_VELOCITY_PID_KP,
        ShooterConstants.SHOOTER_VELOCITY_PID_KI, ShooterConstants.SHOOTER_VELOCITY_PID_KD, 0,
        ShooterConstants.SHOOTER_VELOCITY_PID_TOLERANCE, -12, 12);

    shooterShuffleboard = new Shuffleboard(ShooterConstants.SYSTEM_NAME);

    shooterRightMotor.follow(shooterLeftMotor);

    shooterLeftMotor.configFactoryDefault();
    shooterRightMotor.configFactoryDefault();
    shooterLeftMotor.setSelectedSensorPosition(0);
    // shooterShuffleboard.addNum(sd, 3350);
    // shooterShuffleboard.addNum(s, 3000);
    // shooterShuffleboard.addNum(a, 5);
  }

  public void resetPID() {
    pidController.reset();
  }

  public void setVoltage(double power) {
    shooterLeftMotor.set(ControlMode.PercentOutput, power / 12);
  }

  public double getVelocity() {
    return (((shooterLeftMotor.getSelectedSensorVelocity() + shooterRightMotor.getSelectedSensorVelocity())
     / 2.0) * 600.0) / 1708.5;//RobotConstants.KCTRE_MAG_ENCODER_TPR;
  }

  public void setSetpoint(double setpoint) {
    pidController.setSetpoint(setpoint);
    pidController.setF(((setpoint / 6374) * 12)
        * ShooterConstants.SHOOTER_VELOCITY_PID_KF);
  }

  public double calculate() {
    return pidController.calculate(getVelocity());
  }

  public double calculate(double input) {
    return pidController.calculate(input);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void open() {
    shooterPiston.set(Value.kForward);
  }

  public void close() {
    shooterPiston.set(Value.kReverse);
  }

  public boolean isOpen() {
    return shooterPiston.get() == Value.kForward;
  }

  public static Shooter getInstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }

  public void reset() {
  }

  public boolean canMove() {
    return true;
  }

  public double getStator() {
    return shooterRightMotor.getStatorCurrent();
  }

  /**
   * Get shooter RPM according to the distance from the target
   * 
   * @return RPM setpoint
   */
  public double calculateRPM() {
    if (Limelight.distance() > 2.1) {
      return ((107.98 * Math.pow(frc.robot.Limelight.distance(), 2) - 467.32 * frc.robot.Limelight.distance() + 
      3350)); //3350//3160.2
    }
    return (61.558 * Math.pow(Limelight.distance(), 2) + 6.2312 * Limelight.distance() + 3000); // 3400
  }

  public double calculateAngle() {
    return Limelight.distance() * -1.3097 + 5;
  }

  public double getVoltage() {
    return 0;
  }

  @Override
  public void periodic() {
    // shooterShuffleboard.addNum("Shooter RPM", getVelocity());
    shooterShuffleboard.addBoolean("At Setpoint", atSetpoint());
    // shooterShuffleboard.addNum("Shooter Setpoint", pidController.getSetpoint());
  }

  public void off() {
    shooterPiston.set(Value.kOff);
  }
}
