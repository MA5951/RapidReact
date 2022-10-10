// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.shooter.Shooter;

public class UpperConveyorCommand extends CommandBase {
    /**
     * Upper Conveyor Command With Waiting
     */
    private Conveyor conveyor;

    public UpperConveyorCommand() {
        conveyor = Conveyor.getInstance();
        addRequirements(conveyor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Shooter.getInstance().atSetpoint()) {
            conveyor.setUpperPower(-0.9);
            conveyor.setLowerPower(-0.5);
        } else {
            conveyor.setUpperPower(0);
            conveyor.setLowerPower(0);
        }
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
        return false;
    }
}

