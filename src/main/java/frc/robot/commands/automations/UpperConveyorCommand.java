// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.shooter.Shooter;

public class UpperConveyorCommand extends CommandBase {
    /**
     * Upper Conveyor Command With Waiting
     */
    private Conveyor conveyor;
    private double stator = 0;
    private double timer;

    private boolean ballCounted = false;

    public UpperConveyorCommand() {
        conveyor = Conveyor.getInstance();
        addRequirements(conveyor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ballCounted = false;
        timer = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Shooter.getInstance().atSetpoint()) {
            if (!conveyor.isBallInLower() && !conveyor.isBallInUpper()) {
               timer = Timer.getFPGATimestamp();
            }
            conveyor.setUpperPower(-0.9);

            if (!conveyor.isBallInUpper()) {
                conveyor.setLowerPower(-0.5);
            }

        } 
        else {
            timer = Timer.getFPGATimestamp();
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
        return (!conveyor.isBallInUpper() && !conveyor.isBallInLower()) && (Timer.getFPGATimestamp() - timer) > 0.4;
    }
}
