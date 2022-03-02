package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class ConveyorCommnadAutonomous extends CommandBase {
  private Conveyor conveyor;

  private boolean isBallInlower;
  private boolean isBallInUpper;;
  private double lastCurrent;
  private double time;
  private boolean wasItStack;
  private boolean giveMorePower;
  private boolean isBallMovedFromLower;
  private final double currentDiff = 45;

  public ConveyorCommnadAutonomous() {
    conveyor = Conveyor.getInstance();
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    isBallInlower = false;
    isBallInUpper = false;
    giveMorePower = false;
    isBallMovedFromLower = false;
    lastCurrent = 0;
    if (conveyor.getAmountOfBalls() == 1){
      isBallInUpper = true;
    } else if (conveyor.getAmountOfBalls() == 2){
      isBallInUpper = true;
      isBallInlower = true;
    }
    // conveyor.setAmountOfBalls(0);
  }

  @Override
  public void execute() {
    //System.out.println(conveyor.getUpperStator());
    
    // if (conveyor.isBallInUpper()) {
    //   isBallInUpper = true;
    // }
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
