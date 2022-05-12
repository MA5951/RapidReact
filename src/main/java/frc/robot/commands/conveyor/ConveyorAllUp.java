// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.shooter.Shooter;

public class ConveyorAllUp extends CommandBase {
  /** Creates a new ConveyorAllUp. */

  Conveyor conveyor;

  public ConveyorAllUp() {
    conveyor = conveyor.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Shooter.getInstance().atSetpoint()) {
      conveyor.setUpperPower(-0.45);
      conveyor.setLowerPower(-0.45);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.setUpperPower(0);
    conveyor.setLowerPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
