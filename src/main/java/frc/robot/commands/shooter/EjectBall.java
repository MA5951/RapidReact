// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class EjectBall extends CommandBase {
  /** Creates a new EjectBall. */
  private Shooter shooter;
  private double setpoint;

  public EjectBall(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Shooter.getInstance();
    addRequirements(shooter);
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.open();
    shooter.setSetpoint(this.setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVoltage(shooter.calculate(shooter.getVelocity()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
