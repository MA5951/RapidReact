package frc.robot.commands.conveyor;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class ConveyorCommand extends CommandBase {
  private Conveyor conveyor;

  private boolean isBallInlower;
  private boolean isBallInUpper;
  private double time;
  private boolean wasItStack;
  private boolean giveMorePower;

  public ConveyorCommand() {
    conveyor = Conveyor.getInstance();
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    isBallInlower = false;
    isBallInUpper = false;
    time = Timer.getFPGATimestamp();
    wasItStack = false;
    giveMorePower = false;
    // conveyor.setAmountOfBalls(0);
  }

  @Override
  public void execute() {
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
      if (conveyor.getUpperStator() > 40 && !wasItStack){
        giveMorePower = true;
      } else {
        giveMorePower = false;
      }
      conveyor.setUpperPower(0.5);
      System.out.println("2");
      time = Timer.getFPGATimestamp();
    }
    switch (conveyor.getAmountOfBalls()) {
      case 0:
      case 1:  
        conveyor.setLowerPower(-0.6);
        break;
      case 2:
        conveyor.setLowerPower(0);
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
    return conveyor.isBallInUpper() && (conveyor.getAmountOfBalls() == 2);
  }
}
