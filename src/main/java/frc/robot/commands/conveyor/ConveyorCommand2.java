// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class ConveyorCommand2 extends CommandBase {
  /** Creates a new ConveyorCommand2. */
  private Conveyor conveyor;
  private boolean isBallInLower;
  private double detectedTime = 0;
  private double currentTime;

  public ConveyorCommand2() {
    conveyor = Conveyor.getInstance();
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentTime = Timer.getFPGATimestamp();
    detectedTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if ((conveyor.isBallInLower() && !isBallInLower) && ((currentTime - detectedTime) > 0.2)) {
      detectedTime = Timer.getFPGATimestamp();
      conveyor.setAmountOfBalls(conveyor.getAmountOfBalls() + 1);
    }
    if (conveyor.isBallInUpper) {
      conveyor.setUpperPower(0);
    } else {
      conveyor.setUpperPower(0.6);
    }
    if (conveyor.isBallInUpper()) {
      conveyor.isBallInUpper = true;
    }
    if (conveyor.getAmountOfBalls() == 2) {
      conveyor.setLowerPower(0);
    } else {
      conveyor.setLowerPower(-0.4);
    }

    isBallInLower = conveyor.isBallInLower();
    currentTime = Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.setUpperPower(0);
    conveyor.setLowerPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return conveyor.getAmountOfBalls() == 2 && conveyor.isBallInUpper;
  }
}
