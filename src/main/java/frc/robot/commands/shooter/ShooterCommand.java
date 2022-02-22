// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShooterCommand extends CommandBase {
  /** Creates a new ShooterCommand. */
  private Shooter shooter;

  public ShooterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Shooter.getinstance();
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSetpoint(ShooterConstants.SHOOTER_VELOCITY_LAUNCH_PAD); // Launch Pad: 3275 Fender: 2500
    shooter.resetPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setMotor(shooter.calculate(shooter.getVelocity()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
