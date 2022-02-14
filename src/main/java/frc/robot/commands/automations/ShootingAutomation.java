// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Limelight;
import frc.robot.utils.commands.MApistonCommand;

public class ShootingAutomation extends CommandBase {
  /** Creates a new ShootingAutomation. */
  ShooterCommand shooterCommand;
  UpperConveyorCommand upperConveyorCommand;
  public ShootingAutomation() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Shooter.getinstance(), Conveyor.getinstance());
    
    shooterCommand = new ShooterCommand();
    upperConveyorCommand = new UpperConveyorCommand();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterCommand.initialize();
    upperConveyorCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Limelight.distance() > 0){
      new MApistonCommand(Shooter.getinstance(), true);
    }else{
      new MApistonCommand(Shooter.getinstance(), false);
    }
    
    shooterCommand.execute();

    if (Shooter.getinstance().atSetpoint()){
      upperConveyorCommand.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterCommand.end(false);
    upperConveyorCommand.end(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
