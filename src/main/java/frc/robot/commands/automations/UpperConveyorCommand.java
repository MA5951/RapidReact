// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automations;

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

    private ConveyBallsCommand conveyBallsCommand;


    public UpperConveyorCommand() {
        conveyor = Conveyor.getInstance();
        addRequirements(conveyor);

        conveyBallsCommand = new ConveyBallsCommand();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Shooter.getinstance().atSetpoint()) {
            conveyor.setUpperPower(0.8);
            if (conveyor.getAmountOfBalls() != 2){
                conveyor.setLowerPower(-0.6);
            }

            if (!conveyor.isBallInUpper() && (conveyor.isBallInUpper() != isBallAtTop)) {
                conveyor.setAmountOfBalls(conveyor.getAmountOfBalls() - 1);
            }
        }
        isBallAtTop = conveyor.isBallInUpper();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        conveyBallsCommand.end(true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
