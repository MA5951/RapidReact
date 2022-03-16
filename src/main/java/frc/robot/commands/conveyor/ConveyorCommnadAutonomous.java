package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class ConveyorCommnadAutonomous extends CommandBase {
  private Conveyor conveyor;

  private boolean isBallInlower;
  private boolean isBallInUpper;;

  public ConveyorCommnadAutonomous() {
    conveyor = Conveyor.getInstance();
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    isBallInlower = false;
    isBallInUpper = false;
    if (conveyor.getAmountOfBalls() == 1){
      isBallInUpper = true;
    } else if (conveyor.getAmountOfBalls() == 2){
      isBallInUpper = true;
      isBallInlower = true;
    }
  }

  @Override
  public void execute() {
    conveyor.setUpperPower(0.7);
    conveyor.setLowerPower(-0.6);


    if (conveyor.isBallInLower() && !isBallInlower) {
      conveyor.setAmountOfBalls(conveyor.getAmountOfBalls() + 1);
    }

    if (conveyor.isBallInUpper()) {
      isBallInUpper = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    conveyor.setLowerPower(0);
    conveyor.setUpperPower(0);
  }

  @Override
  public boolean isFinished() {
    return isBallInUpper;
  }
}
