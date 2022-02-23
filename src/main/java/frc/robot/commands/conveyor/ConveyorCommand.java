package frc.robot.commands.conveyor;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class ConveyorCommand extends CommandBase {

  boolean ballIn = false;

  private Conveyor conveyor;

  private double time;


  public  ConveyorCommand() {
    conveyor = Conveyor.getInstance();
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
    //conveyor.setAmountOfBalls(0);
  }

  @Override
  public void execute() {
    if (conveyor.isBallInLower()) {
      ballIn = true;
    } else if (ballIn && Timer.getFPGATimestamp() - time >= 0.5) {
      conveyor.setAmountOfBalls(conveyor.getAmountOfBalls() + 1);
      time = Timer.getFPGATimestamp();
      ballIn = false;
    }
    if (conveyor.getAmountOfBalls() == 2){
      conveyor.setLowerPower(0);
    } else {
      conveyor.setLowerPower(-0.4);
    }
    if (conveyor.isBallInUpper()){
      conveyor.setUpperPower(0);
    } else {
      conveyor.setUpperPower(0.5);
    }
  }

  @Override
  public void end(boolean interrupted) {
    conveyor.setLowerPower(0);
    conveyor.setUpperPower(0);
  }

  @Override
  public boolean isFinished() {
    return conveyor.isBallInUpper() && (conveyor.getAmountOfBalls() == 2);
  }
}
