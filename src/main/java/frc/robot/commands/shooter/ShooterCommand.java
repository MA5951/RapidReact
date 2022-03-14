// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShooterCommand extends CommandBase {
  /** Creates a new ShooterCommand. */
  private Shooter shooter;
  private Supplier<Double> setpoint;
  private boolean stop;
  private double timeOut;
  private double time;

  public ShooterCommand(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Shooter.getInstance();
    addRequirements(shooter);
    this.setpoint = () -> setpoint;
    this.stop = true;
  }

  public ShooterCommand(Supplier<Double> setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Shooter.getInstance();
    addRequirements(shooter);
    this.setpoint = setpoint;
    this.stop = true;
  }

  public ShooterCommand(double setpoint, boolean stop) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(setpoint);
    this.stop = stop;
  }

  public ShooterCommand(Supplier<Double> setpoint, boolean stop) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(setpoint);
    this.stop = stop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Limelight.getTv()) {
      if (Limelight.distance() > 2.1) {
        shooter.close();
      } else {
        shooter.open();
      }
      shooter.setSetpoint(setpoint.get()); // Launch Pad: 3275 Fender: 2500
    } else {
      shooter.open();
      shooter.setSetpoint(ShooterConstants.SHOOTER_VELOCITY_FENDER);
    }
    shooter.setVoltage(shooter.calculate(shooter.getVelocity()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
