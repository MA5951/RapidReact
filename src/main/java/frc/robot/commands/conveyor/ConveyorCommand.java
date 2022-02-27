package frc.robot.commands.conveyor;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class ConveyorCommand extends CommandBase {
  private Conveyor conveyor;

  private boolean isBallInlower;
  private boolean isBallInUpper;;
  private double lastCurrent;
  private double time;
  private boolean wasItStack;
  private boolean giveMorePower;
  private boolean isBallMovedFromLower;
  private final double currentDiff = 45;

  public ConveyorCommand() {
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

    if (conveyor.isBallInLower() && !isBallInlower) {
      conveyor.setAmountOfBalls(conveyor.getAmountOfBalls() + 1);
    }

    if (conveyor.isBallInUpper()) {
      isBallInUpper = true;
    }
    if (isBallInUpper) {
      conveyor.setUpperPower(0);
    } else if (giveMorePower && Timer.getFPGATimestamp() - time <= 0.15) {
      conveyor.setUpperPower(1);
      System.out.println("1");
      wasItStack = true;

    } else {
      conveyor.setUpperPower(0.6);
      if (conveyor.getUpperStator() > 40 && !wasItStack){
        giveMorePower = true;
      } else {
        giveMorePower = false;
      }
      conveyor.setUpperPower(0.5);
      time = Timer.getFPGATimestamp();
    }
    switch (conveyor.getAmountOfBalls()) {
      case 0:
      case 1:
      conveyor.setLowerPower(-0.6);
      break;
      case 2:
        conveyor.setLowerPower(0);
        break;
    }
    isBallInlower = conveyor.isBallInLower();
  }

  @Override
  public void end(boolean interrupted) {
    conveyor.setLowerPower(0);
    conveyor.setUpperPower(0);
  }

  @Override
  public boolean isFinished() {
    return (conveyor.getAmountOfBalls() == 2);
  }
}
