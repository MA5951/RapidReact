// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climb.ClimbExtension;

public class CloseExtensionArm extends CommandBase {
  /** Creates a new CloseExtensionArm. */
  private ClimbExtension climbExtension;
  private final static double maxCurrent = 8;

  public CloseExtensionArm(ClimbExtension climbExtension) {
    this.climbExtension = climbExtension;

    addRequirements(this.climbExtension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbExtension.setPower(-0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbExtension.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbExtension.getCurrent() < maxCurrent;
  }
}
