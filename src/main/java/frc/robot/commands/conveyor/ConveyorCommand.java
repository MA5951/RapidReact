package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class ConveyorCommand extends CommandBase {

  boolean ballIn = false;

  private Conveyor conveyor;


  public ConveyorCommand() {
    conveyor = Conveyor.getInstance();
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    if (conveyor.isBallInLower()){
      conveyor.setAmountOfBalls(1);
    }

  }

  @Override
  public void execute() {
    if (conveyor.getAmountOfBalls() == 2)
      conveyor.setLowerPower(0); //0
    else
      conveyor.setLowerPower(0.75);

    if (conveyor.isBallInUpper()){
      conveyor.setUpperPower(0); //0
    }
    else{
      conveyor.setUpperPower(-0.75);
    }


  


    if (conveyor.isBallInLower()) {
      ballIn = true;
    }
    
    else if (ballIn) {
      conveyor.setAmountOfBalls(conveyor.getAmountOfBalls()+1);
      ballIn = false;
    }


    SmartDashboard.putNumber("amoutOfBalls", conveyor.getAmountOfBalls());
  }

  @Override
  public void end(boolean interrupted) {
    conveyor.setLowerPower(0);
    conveyor.setUpperPower(0);;
  }

  @Override
  public boolean isFinished() {
    return conveyor.getAmountOfBalls() == 2;
  }
}
