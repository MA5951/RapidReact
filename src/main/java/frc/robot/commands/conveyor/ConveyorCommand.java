package frc.robot.commands.conveyor;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class ConveyorCommand extends CommandBase {
  private Conveyor conveyor;

  private boolean isBallInlower;
  private double time1;
  private double time2;
  private double time3;
  private boolean isBallInUpper;

  public ConveyorCommand() {
    conveyor = Conveyor.getInstance();
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    time1 = Timer.getFPGATimestamp();
    time2 = Timer.getFPGATimestamp();
    time3 = Timer.getFPGATimestamp();
    isBallInlower = false;
    isBallInUpper = false;
    // conveyor.setAmountOfBalls(0);
  }

  @Override
  public void execute() {
    if (conveyor.isBallInLower() && !isBallInlower){
      conveyor.setAmountOfBalls(conveyor.getAmountOfBalls() + 1);
    }
    if (conveyor.isBallInUpper()){
      isBallInUpper = true;
    }
    if (isBallInUpper) {
      conveyor.setUpperPower(0);
    } else {
      conveyor.setUpperPower(0.7);
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
