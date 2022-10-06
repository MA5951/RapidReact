// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import java.sql.Time;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Automations.IntakeAutomation;
import frc.robot.commands.shooter.EjectBall;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ConveyorCommandColor extends CommandBase {
  /** Creates a new ConveyorCommandColor. */
  private Conveyor conveyor;
  private double lastTimeInLower;
  private double lastTimeInUpper;
  private double whenEntered;
  private Supplier<Boolean> isOurTeam;
  private Supplier<Boolean> isEnemyTeam;
  private boolean isEnemyBallInLower;
  private boolean isEnemyBallInUpper;


  public ConveyorCommandColor(Conveyor conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.conveyor = conveyor;
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Conveyor.getInstance().getOurTeamColorBlue()) {
      isOurTeam = Conveyor.getInstance()::isBlue;
      isEnemyTeam = Conveyor.getInstance()::isRed;
    } else {
      isOurTeam = Conveyor.getInstance()::isRed;
      isEnemyTeam = Conveyor.getInstance()::isBlue;
    }
    isEnemyBallInLower = false;
    isEnemyBallInUpper = false;
    lastTimeInLower = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    if (isEnemyTeam.get() && !conveyor.isBallInUpper()) {
      isEnemyBallInUpper = true;
      lastTimeInUpper = Timer.getFPGATimestamp();
      Shooter.getInstance().setPower(0.25);
    }

    if (Timer.getFPGATimestamp() - lastTimeInUpper > 2 && isEnemyBallInUpper) {
      isEnemyBallInUpper = false;
      Shooter.getInstance().setPower(0);
    }

    if (isEnemyTeam.get() && conveyor.isBallInUpper()) {
      isEnemyBallInLower = true;
      lastTimeInLower = Timer.getFPGATimestamp();
    }

    if (Timer.getFPGATimestamp() - lastTimeInLower > 0.8 && isEnemyBallInLower) {
      isEnemyBallInLower = false;
    }

    if(conveyor.isBallInUpper() && !isEnemyBallInUpper){
      conveyor.setUpperPower(0);
    } else {
      conveyor.setUpperPower(-0.5);
    }
    if(isEnemyBallInLower && conveyor.isBallInUpper()){
      conveyor.setLowerPower(0.5);
      Intake.getInstance().setPower(-0.8);
    } else {
      conveyor.setLowerPower(-0.3);
      Intake.getInstance().setPower(0.8);
    }











  }
    // if (isEnemyBall && conveyor.isBallInUpper()) {
    //   conveyor.setLowerPower(0.7);
    //   conveyor.setUpperPower(0);
    //   Intake.getInstance().setPower(-0.6);
    //   Shooter.getInstance().setPower(0);
    // } else if (isEnemyBall) {
    //   conveyor.setLowerPower(-0.5);
    //   Intake.getInstance().setPower(0.5);
    //   conveyor.setUpperPower(-0.5);
    //   Shooter.getInstance().setPower(0.2);
    // } else {
    //   Shooter.getInstance().setPower(0);
    //   lastTimeInLower = Timer.getFPGATimestamp();
    //   conveyor.setLowerPower(-0.5);
    //   Intake.getInstance().setPower(0.5);
    //   if (!conveyor.isBallInUpper()) {
    //     conveyor.setUpperPower(-0.5);
    //   } else {
    //     conveyor.setUpperPower(0);
    //   }
    // }

    // if (conveyor.isBallInLower()) {
    // if (conveyor.isBallInUpper()) {
    // conveyor.setLowerPower(0);
    // conveyor.setUpperPower(0);
    // } else {
    // if (!conveyor.isBlue()) {
    // conveyor.setUpperPower(-0.5);
    // conveyor.setLowerPower(-0.3);
    // new EjectBall(-1500);
    // } else {
    // conveyor.setUpperPower(-0.5);
    // conveyor.setLowerPower(-0.3);
    // }
    // }
    // } else {
    // if (conveyor.isBallInUpper) {
    // if (!conveyor.isBlue()) {
    // new IntakeAutomation(0.8);
    // conveyor.setLowerPower(0.3);
    // }
    // }
    // }

  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.setLowerPower(0);
    conveyor.setUpperPower(0);
    Intake.getInstance().setPower(0);
    Shooter.getInstance().setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isOurTeam.get() && conveyor.isBallInUpper();
  }
}