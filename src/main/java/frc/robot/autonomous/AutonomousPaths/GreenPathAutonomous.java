// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.autonomous;
package frc.robot.autonomous.AutonomousPaths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.Paths;
import frc.robot.commands.Automations.IntakeAutomation;
import frc.robot.commands.Automations.UpperConveyorCommand;
import frc.robot.commands.chassis.AutonomousCommand;
import frc.robot.commands.chassis.PIDVision;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GreenPathAutonomous extends SequentialCommandGroup {
    /**
     * Two Balls Autonomous
     */
    public GreenPathAutonomous() {
        addCommands(
                new ParallelDeadlineGroup(
                        new WaitCommand(3),
                        new AutonomousCommand(Paths.gettingOutOfLunchPad, false),
                         new IntakeAutomation(0.8)
                ),
                new PIDVision(Shooter.getInstance().calculateAngle()),
                new ParallelDeadlineGroup(
                        new WaitCommand(3),
                        new ShooterCommand(() -> Shooter.getInstance().calculateRPM()),
                        new UpperConveyorCommand()
                )
        );
    }
}