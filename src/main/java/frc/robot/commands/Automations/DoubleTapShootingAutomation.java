// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.chassis.PIDVision;
import frc.robot.subsystems.shooter.Shooter;

public class DoubleTapShootingAutomation extends CommandBase {
  /** Creates a new DoubleTapShootingAutomation. */
  private static int state;
  public DoubleTapShootingAutomation() {
    // Use addRequirements() here to declare subsystem dependencies.
    state = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state++;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (state == 1){
      new PIDVision(Shooter.getInstance().calculateAngle());
    }else if (state == 2){
      new ShootingOnlyAutomation();
    }

    System.out.println(state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (state == 2){
      state = 0;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
