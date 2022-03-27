// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class ConveyorCommand extends CommandBase {
  /** Creates a new ConveyorCommand2. */
  private Conveyor conveyor;

  public ConveyorCommand() {
    conveyor = Conveyor.getInstance();
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (conveyor.isBallInUpper()) {
      conveyor.setUpperPower(0);
    }
    else {
      conveyor.setUpperPower(-0.5);
      conveyor.setLowerPower(-0.3);
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
    return conveyor.isBallInLower() && conveyor.isBallInUpper();
  }
}
