package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;

public class ConveyBallsCommand extends CommandBase {
    /**
     *
     */

    private final double conveyorPower = ConveyorConstants.CONVEYOR_POWER;
    private Conveyor conveyor = Conveyor.getInstance();

    public ConveyBallsCommand() {
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        if (conveyor.isBallInLower()) {
            conveyor.setAmountOfBalls(conveyor.getAmountOfBalls() + 1);
        }
        if (conveyor.isBallInUpper()) {
            conveyor.setAmountOfBalls(conveyor.getAmountOfBalls() + 1);
        }
    }

    @Override
    public void execute() {
        conveyor.setUpperPower(0.75);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setLowerPower(0);
        conveyor.setUpperPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
