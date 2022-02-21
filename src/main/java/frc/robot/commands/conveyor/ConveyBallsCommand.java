package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;

public class ConveyBallsCommand extends CommandBase {

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
        if (conveyor.isBallInLower()) {
            conveyor.setAmountOfBalls(conveyor.getAmountOfBalls() + 1);
        }

        if (conveyor.isBallInUpper()) {
            conveyor.setLowerPower(conveyorPower);
            conveyor.setUpperPower(conveyorPower); //0
        } else {
            conveyor.setLowerPower(conveyorPower);
            conveyor.setUpperPower(conveyorPower);
        }

        SmartDashboard.putNumber("amountOfBalls", conveyor.getAmountOfBalls());
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setLowerPower(0);
        conveyor.setUpperPower(0);
    }

    @Override
    public boolean isFinished() {
        return conveyor.getAmountOfBalls() == ConveyorConstants.CONVEYOR_MAX_BALLS;
    }
}
