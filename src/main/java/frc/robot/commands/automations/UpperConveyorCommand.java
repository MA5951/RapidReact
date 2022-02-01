// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automations;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.conveyor.ConveyorCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.shooter.Shooter;

public class UpperConveyorCommand extends CommandBase {
  /** Creates a new UpperConveyorCommand. */
  private Conveyor conveyor;

  private ConveyorCommand conveyorTransportCommand;

  public UpperConveyorCommand() {
    conveyor = Conveyor.getinstance();
    addRequirements(conveyor);

    conveyorTransportCommand = new ConveyorCommand();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyorTransportCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Shooter.getinstance().atSetpoint()) {
      conveyor.setLowerPower(-0.6);
      conveyor.setUpperPower(0.6);
      conveyor.amoutOfBalls--;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyorTransportCommand.end(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
