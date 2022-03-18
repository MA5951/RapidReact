// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climb.ClimbExtension;

public class ClimbToSecend extends CommandBase {
  /** Creates a new ClimbToSecend. */
  public ClimbToSecend() {
     addRequirements(ClimbExtension.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClimbExtension.getInstance().setSetpoint(-0.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ClimbExtension.getInstance().setVoltage(ClimbExtension.getInstance().calculate());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimbExtension.getInstance().setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ClimbExtension.getInstance().getCurrent() > 108;
  }
}
