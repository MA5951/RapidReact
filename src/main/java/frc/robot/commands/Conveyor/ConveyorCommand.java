package frc.robot.commands.Conveyor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ConveyorCommand extends CommandBase {

  boolean ballIn = false;

  private Conveyor conveyor;


  public ConveyorCommand() {
    conveyor = Conveyor.getinstance();
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    if (conveyor.getIR1()){
      conveyor.amoutOfBalls = 1;
    }

  }

  @Override
  public void execute() {
    if (conveyor.amoutOfBalls == 2)
      conveyor.setLowerPower(0); //0
    else
      conveyor.setLowerPower(0.75);

    if (conveyor.getIR2()){
      conveyor.setUpperPower(0); //0
    }
    else{
      conveyor.setUpperPower(-0.75);
    }


  


    if (conveyor.getIR1()) {
      ballIn = true;
    }
    
    else if (ballIn) {
      conveyor.amoutOfBalls++;
      ballIn = false;
    }


    SmartDashboard.putNumber("amoutOfBalls", conveyor.amoutOfBalls);
  }

  @Override
  public void end(boolean interrupted) {
    conveyor.setLowerPower(0);
    conveyor.setUpperPower(0);;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
