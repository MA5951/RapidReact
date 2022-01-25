package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ConveyorCommand extends CommandBase {

  int amoutOfBalls=0;

  boolean ballIn = false;

  private Conveyor conveyor;


  public ConveyorCommand() {
    conveyor = Conveyor.getinstance();
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (amoutOfBalls == 2)
      conveyor.setTalon1Velocity(0);
    else
      conveyor.setTalon1Velocity(0.5);

    if (conveyor.getIR2())
      conveyor.setTalon2Velocity(0);
    else
      conveyor.setTalon2Velocity(0.5);


  


    if (conveyor.getIR1()) {
      ballIn = true;
    }
    
    else if (ballIn) {
      amoutOfBalls++;
      ballIn = false;
    }


    SmartDashboard.putNumber("amoutOfBalls", amoutOfBalls);
  }

  @Override
  public void end(boolean interrupted) {
    conveyor.setTalon1Velocity(0);
    conveyor.setTalon2Velocity(0);;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConveyorCommand extends CommandBase {
  /** Creates a new ConveyorCommand. */
  public ConveyorCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
