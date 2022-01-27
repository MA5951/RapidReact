package frc.robot.commands.Conveyor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ConveyorCommand extends CommandBase {

  int amoutOfBalls = 0;

  boolean ballIn = false;

  private Conveyor conveyor;


  public ConveyorCommand() {
    conveyor = Conveyor.getinstance();
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    if (conveyor.getIR1()){
      amoutOfBalls++;
    }

  }

  @Override
  public void execute() {
    if (amoutOfBalls == 2)
      conveyor.setLowerPower(0);
    else
      conveyor.setLowerPower(-0.7);

    if (conveyor.getIR2())
      conveyor.setUpperPower(0);
    else
      conveyor.setUpperPower(0.7);


  


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
    conveyor.setLowerPower(0);
    conveyor.setUpperPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
