// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automations;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.conveyor.ConveyBallsCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.shooter.Shooter;

public class UpperConveyorCommand extends CommandBase {
    /**
     * Creates a new UpperConveyorCommand.
     */
    private Conveyor conveyor;

    private boolean isBallAtTop = true;
    private boolean isStuck = false;

    private double time;

    private ConveyBallsCommand conveyBallsCommand;

    public UpperConveyorCommand() {
        conveyor = Conveyor.getInstance();
        addRequirements(conveyor);

        conveyBallsCommand = new ConveyBallsCommand();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        time = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Shooter.getinstance().atSetpoint()) {
            // if (!isStuck) {
            //     time = Timer.getFPGATimestamp();
            // }
            if (Timer.getFPGATimestamp() - time <= 0.6){
                conveyor.setUpperPower(-0.4);
            } else {
                conveyor.setUpperPower(0.9);
            }
            // // if (conveyor.getUpperCurrent() > 50) {// && (Timer.getFPGATimestamp() -
            // time <= 1)){
            // // isStuck = true;
            // // conveyor.setUpperPower(-0.4);
            // // time = Timer.getFPGATimestamp();
            // } else {
            // isStuck = false;
            // conveyor.setUpperPower(0.9);
            // }
            if (conveyor.getAmountOfBalls() != 2) {
                conveyor.setLowerPower(-0.6);
            }
            if (!conveyor.isBallInUpper() && (conveyor.isBallInUpper() != isBallAtTop)) {
                conveyor.setAmountOfBalls(conveyor.getAmountOfBalls() - 1);
            }
            isBallAtTop = conveyor.isBallInUpper();
        } else 
            conveyor.setUpperPower(0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        conveyor.setLowerPower(0);
        conveyor.setUpperPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;//conveyor.getAmountOfBalls() == 0;
    }
}
