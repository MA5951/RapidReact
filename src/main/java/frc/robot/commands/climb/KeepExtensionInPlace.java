// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climb.ClimbExtension;

public class KeepExtensionInPlace extends CommandBase {
  /** Creates a new KeepExtensionInPlace. */

  ClimbExtension climbExtension;

  public KeepExtensionInPlace() {
    climbExtension = ClimbExtension.getInstance();
    addRequirements(climbExtension);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbExtension.keepArmInPlace();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbExtension.setVoltage(climbExtension.calculate());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbExtension.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbExtension.atSetpoint();
  }
}
