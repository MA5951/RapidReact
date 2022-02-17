// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommand extends CommandBase {
  /** Creates a new ShooterCommand. */
  private Shooter shooter;
  private double setpoint;
  private boolean stop;

  public ShooterCommand(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Shooter.getinstance();
    addRequirements(shooter);
    this.setpoint = setpoint;
    this.stop = true;
  }

  public ShooterCommand(double setpoint, boolean stop) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Shooter.getinstance();
    addRequirements(shooter);
    this.setpoint = setpoint;
    this.stop = stop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSetpoint(setpoint); //Launch Pad: 3275 Fender: 2500
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setMotor(shooter.calculate(shooter.getEncoder()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stop) {
      shooter.setMotor(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
